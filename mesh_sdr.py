#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""
Meshtastic SDR — decodificador de pacotes Meshtastic com RTL-SDR, cruzando com um módulo via USB.

Exemplos:
  python mesh_sdr.py                                  # 919,875 MHz LongFast, módulo detectado sozinho
  python mesh_sdr.py --porta COM5 --ganho 38.6
  python mesh_sdr.py --sem-modulo --canal "Familia:base64DaPSK=="
  python mesh_sdr.py --arquivo captura.cu8 --taxa 1000000 --sem-modulo
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import os
import sys
import time
from datetime import datetime

import numpy as np

from lora_phy import LoRaReceiver, time_on_air_ms
from mesh_decode import (BROADCAST, PRESETS, ChannelKey, KeyRing, NodeDB, data_meta, decode_content,
                         decrypt, expand_psk, node_id, parse_header, parse_psk_arg, port_name)
from sources import FileSource, FrontEnd, RtlSource, plan_rates

try:
    from rich.console import Console
    from rich.panel import Panel
    from rich.text import Text
    console = Console(highlight=False)
except ImportError:  # pragma: no cover
    console = None


def say(msg: str, style: str = ""):
    if console:
        console.print(msg, style=style, markup=False)
    else:
        print(msg)


# ============================================================================
# Formatação
# ============================================================================

NODE_KEYS = {"node_id", "from", "to", "dest", "source", "last_sent_by_id", "route", "route_back",
             "num", "last_heard_by", "requester", "dst"}


def fmt_val(k: str, v, db: NodeDB):
    if k in NODE_KEYS:
        if isinstance(v, list):
            return " → ".join(db.label(int(x)) for x in v) if v else "(vazio)"
        try:
            return db.label(int(v))
        except (TypeError, ValueError):
            return v
    if k in ("snr_towards", "snr_back") and isinstance(v, list):
        return ", ".join(f"{x / 4:.2f} dB" for x in v)
    if k in ("time", "timestamp", "rx_time", "last_heard") and isinstance(v, (int, float)) and v > 1e9:
        return f"{v} ({datetime.fromtimestamp(v).strftime('%d/%m/%Y %H:%M:%S')})"
    if k in ("uptime_seconds",) and isinstance(v, (int, float)):
        d, r = divmod(int(v), 86400)
        h, r = divmod(r, 3600)
        return f"{v} s ({d}d {h}h {r // 60}min)"
    return v


def flatten(d, db: NodeDB, prefix="", out=None):
    if out is None:
        out = []
    if isinstance(d, dict):
        for k, v in d.items():
            if isinstance(v, dict):
                flatten(v, db, f"{prefix}{k}.", out)
            elif isinstance(v, list) and v and isinstance(v[0], dict):
                for i, item in enumerate(v):
                    flatten(item, db, f"{prefix}{k}[{i}].", out)
            else:
                out.append((prefix + k, fmt_val(k, v, db)))
    return out


def enrich(content: dict, db: NodeDB, h) -> list[tuple[str, str]]:
    """Linhas extras derivadas para portas conhecidas."""
    extra = []
    f = content.get("campos", {})
    p = content.get("porta")
    if p == "POSITION_APP" and "latitude_i" in f:
        lat, lon = f["latitude_i"] * 1e-7, f.get("longitude_i", 0) * 1e-7
        extra.append(("posição", f"{lat:.6f}, {lon:.6f}"))
        extra.append(("mapa", f"https://www.openstreetmap.org/?mlat={lat:.6f}&mlon={lon:.6f}#map=15/{lat:.5f}/{lon:.5f}"))
        if "precision_bits" in f and 0 < f["precision_bits"] < 32:
            err_m = 23905787.925008 / (2 ** f["precision_bits"])  # tamanho aproximado da célula
            extra.append(("precisão", f"{f['precision_bits']} bits (~{err_m:.0f} m)"))
        me = db.nodes.get(db.my_num) if db.my_num else None
        if me and me.position and h.frm != db.my_num:
            dkm = haversine(me.position[0], me.position[1], lat, lon)
            extra.append(("distância do seu nó", f"{dkm:.2f} km"))
        db.get(h.frm).position = (lat, lon)
    if p == "ROUTING_APP":
        er = f.get("error_reason")
        if er is not None:
            extra.append(("resultado", "ACK (entregue)" if er == "NONE" else f"NAK: {er}"))
        elif "route_request" in f or "route_reply" in f:
            extra.append(("resultado", "descoberta de rota"))
        else:
            extra.append(("resultado", "ACK (entregue)"))
    if p == "TELEMETRY_APP":
        dm = f.get("device_metrics", {})
        if "battery_level" in dm:
            b = dm["battery_level"]
            extra.append(("bateria", "alimentado (USB/externo)" if b > 100 else f"{b}%"))
    return extra


def haversine(la1, lo1, la2, lo2):
    r = 6371.0088
    p1, p2 = math.radians(la1), math.radians(la2)
    dp, dl = p2 - p1, math.radians(lo2 - lo1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


# ============================================================================
# Aplicação
# ============================================================================


class App:
    def __init__(self, args):
        self.a = args
        self.db = NodeDB()
        self.ring = KeyRing()
        self.module = None
        self.count = 0
        self.stats = dict(quadros=0, crc_ok=0, crc_ruim=0, meshtastic=0, decifrados=0, cifrados=0,
                          modulo_rx=0, modulo_e_sdr=0, so_modulo=0, so_sdr=0)
        self.sdr_seen: dict[tuple, list] = {}       # (from,id) -> [nº do quadro, ...]
        self.mod_seen: dict[tuple, dict] = {}
        self.mod_pending: list[tuple[float, tuple]] = []
        self.log = open(args.log, "a", encoding="utf-8") if args.log else None
        self.t0_wall = None
        self.last_stats = time.time()

    # ------------------------------------------------------------ configuração
    def setup(self):
        a = self.a
        if a.preset not in PRESETS:
            sys.exit(f"preset desconhecido: {a.preset}. Opções: {', '.join(PRESETS)}")
        bw, sf, cr = PRESETS[a.preset]
        self.bw = a.bw * 1e3 if a.bw else bw
        self.sf = a.sf or sf
        self.cr = a.cr or cr
        ts = (1 << self.sf) / self.bw
        self.ldro = a.ldro if a.ldro is not None else ts >= 0.016

        # chave padrão do preset (sempre)
        self.ring.add(ChannelKey(a.preset, expand_psk(b"\x01"), False, "padrão"))
        if a.preset != "LongFast":
            self.ring.add(ChannelKey("LongFast", expand_psk(b"\x01"), False, "padrão"))

        if not a.sem_modulo and not a.arquivo_somente:
            try:
                say(f"Conectando ao módulo Meshtastic {a.porta or '(detecção automática)'}...", "cyan")
                from module_link import ModuleLink
                self.module = ModuleLink(a.porta)
                chans = self.module.load_keys(self.ring)
                n = self.module.load_nodes(self.db)
                say(f"  módulo {self.db.label(self.db.my_num) if self.db.my_num else ''} em {self.module.port}", "green")
                lc = self.module.lora_config()
                say(f"  LoRa do módulo: região {lc.get('region', '?')}, preset {self.module.preset_name()}, "
                    f"slot {lc.get('channel_num', 0) or 'padrão'}"
                    + (f", freq. forçada {lc['override_frequency']} MHz" if lc.get('override_frequency') else ""), "green")
                if self.module.preset_name() not in (a.preset, "Custom"):
                    say(f"  [aviso] o módulo usa {self.module.preset_name()} mas o SDR está em {a.preset}", "bold yellow")
                say(f"  {len(chans)} canal(is) e {n} nós importados; chave privada PKI: "
                    f"{'sim' if self.ring.private_key else 'não disponível'}", "green")
            except Exception as e:  # noqa: BLE001
                say(f"  [aviso] não consegui abrir o módulo: {e}", "bold yellow")
                say("  Seguindo só com o SDR. Se a porta existir mas estiver ocupada, feche o app Meshtastic/navegador;"
                    " para escolher a porta use --porta COMx (veja --listar-portas).", "yellow")
                self.module = None

        for spec in a.canal or []:
            parts = spec.split(":")
            name, psk = parts[0], (parts[1] if len(parts) > 1 else "default")
            aead = len(parts) > 2 and parts[2].lower() in ("aead", "ccm")
            self.ring.add(ChannelKey(name, expand_psk(parse_psk_arg(psk)), aead, "linha de comando"))
        if a.chave_privada:
            self.ring.private_key = base64.b64decode(a.chave_privada)
        if a.meu_no:
            self.db.my_num = int(a.meu_no.lstrip("!"), 16)

        say("Chaves de canal:", "bold")
        for c in self.ring.channels:
            say(f"  • {c.describe()}")

    # ------------------------------------------------------------ eventos do módulo
    def poll_module(self):
        if not self.module:
            return
        while not self.module.events.empty():
            kind, t, pkt = self.module.events.get()
            if kind == "lost":
                say("[módulo] conexão perdida", "bold red")
                continue
            key = (pkt.get("from"), pkt.get("id"))
            if key[0] is None or key[1] is None:
                continue
            self.stats["modulo_rx"] += 1
            dec = pkt.get("decoded", {})
            info = dict(t=t, rssi=pkt.get("rxRssi"), snr=pkt.get("rxSnr"), port=dec.get("portnum"),
                        hop_limit=pkt.get("hopLimit"), hop_start=pkt.get("hopStart"),
                        relay=pkt.get("relayNode"), pki=pkt.get("pkiEncrypted", False),
                        text=dec.get("text"))
            first = key not in self.mod_seen
            self.mod_seen[key] = info
            # aprende nomes/chaves com o que o módulo decodifica
            if dec.get("portnum") == "NODEINFO_APP" and "user" in dec:
                u = dec["user"]
                self.db.update_user(key[0], {"long_name": u.get("longName", ""), "short_name": u.get("shortName", ""),
                                             "hw_model": u.get("hwModel", ""), "public_key": u.get("publicKey", "")},
                                    "módulo")
            if not first:
                continue
            if key in self.sdr_seen:
                self.stats["modulo_e_sdr"] += 1
                nums = ", ".join(f"#{n}" for n in self.sdr_seen[key])
                say(f"   ✓ módulo também recebeu {nums}: RSSI {info['rssi']} dBm, SNR {info['snr']} dB", "green")
            else:
                self.mod_pending.append((t, key))
        # pacotes que o módulo viu e o SDR não, depois de um tempo de espera
        now = time.time()
        keep = []
        for t, key in self.mod_pending:
            if key in self.sdr_seen:
                self.stats["modulo_e_sdr"] += 1
                continue
            if now - t > self.a.espera_modulo:
                self.stats["so_modulo"] += 1
                i = self.mod_seen[key]
                desc = i["port"] or "?"
                if i.get("text"):
                    desc += f' "{i["text"]}"'
                say(f"   ⚠ só o módulo recebeu: {self.db.label(key[0])} id 0x{key[1]:08x} {desc} "
                    f"(RSSI {i['rssi']} dBm, SNR {i['snr']} dB) — o SDR não captou", "yellow")
            else:
                keep.append((t, key))
        self.mod_pending = keep

    # ------------------------------------------------------------ quadro decodificado
    def frame_time(self, fr) -> str:
        secs = fr.start_sample / self.bw
        if self.a.arquivo:
            return f"t+{secs:9.3f}s"
        return datetime.fromtimestamp(self.t0_wall + secs).strftime("%H:%M:%S.%f")[:-3]

    def handle(self, fr):
        a = self.a
        self.stats["quadros"] += 1
        pl = fr.payload
        crc_ok = pl.crc_ok if pl else None
        self.stats["crc_ok" if crc_ok else "crc_ruim"] += 1
        if not crc_ok and not a.mostrar_falhas:
            return
        self.count += 1
        n = self.count
        raw = pl.data if pl else b""
        lines: list[tuple[str, str]] = []
        toa = time_on_air_ms(fr.header.length, self.sf, self.bw, fr.header.cr, fr.preamble_syms, fr.header.has_crc, self.ldro)
        lines += [("— RF", ""),
                  ("frequência", f"{(self.freq_hz + fr.cfo_hz) / 1e6:.6f} MHz (desvio {fr.cfo_hz:+.0f} Hz"
                                 f" / {fr.cfo_hz / self.freq_hz * 1e6:+.2f} ppm)"),
                  ("SNR estimado", f"{fr.snr_db:.1f} dB"),
                  ("nível", f"{fr.level_dbfs:.1f} dBFS"),
                  ("tempo no ar", f"{toa:.1f} ms ({fr.preamble_syms} símbolos de preâmbulo + 4,25 + {fr.n_syms} de dados)")]
        sw = f"0x{fr.sync_word:02X}" + ("" if fr.sync_word == 0x2B else "  ← diferente do Meshtastic (0x2B)")
        lines += [("— LoRa", ""),
                  ("parâmetros", f"SF{self.sf} BW{self.bw / 1e3:g} kHz CR 4/{fr.header.cr + 4} "
                                 f"LDRO {'sim' if self.ldro else 'não'}"),
                  ("sync word", sw),
                  ("cabeçalho", f"{fr.header.length} bytes, CRC {'presente' if fr.header.has_crc else 'ausente'}, "
                                f"checksum ok, {fr.header.corrected_bits} bit(s) corrigido(s)"),
                  ("CRC payload", "OK" if crc_ok else (f"FALHOU (recebido {pl.crc_rx:#06x}, calculado {pl.crc_calc:#06x})"
                                                       if pl and pl.crc_rx is not None else "n/d")),
                  ("FEC", f"{pl.corrected_bits if pl else 0} bit(s) corrigido(s) no payload"
                          + (f", {pl.chase_fixed} símbolo(s) recuperado(s) pelo CRC" if pl and pl.chase_fixed else "")),
                  ("deriva no pacote", f"{fr.drift_bins * self.bw / (1 << self.sf):+.1f} Hz/símbolo "
                                       f"({fr.drift_bins * fr.n_syms * self.bw / (1 << self.sf):+.0f} Hz do início ao fim)")]

        record = dict(n=n, hora=self.frame_time(fr).strip(), unix=time.time(),
                      rf=dict(freq_hz=round(float(self.freq_hz + fr.cfo_hz), 1), cfo_hz=round(float(fr.cfo_hz), 1),
                              snr_db=round(float(fr.snr_db), 2), nivel_dbfs=round(float(fr.level_dbfs), 2),
                              toa_ms=round(float(toa), 1)),
                      lora=dict(sf=self.sf, bw=self.bw, cr=fr.header.cr, len=fr.header.length, crc_ok=crc_ok,
                                sync_word=fr.sync_word, fec_bits=pl.corrected_bits if pl else None),
                      bruto_hex=raw.hex())
        title_port, style = "?", "white"
        h = parse_header(raw) if crc_ok else None
        if h:
            self.stats["meshtastic"] += 1
            key = (h.frm, h.id)
            repeat = key in self.sdr_seen
            self.sdr_seen.setdefault(key, []).append(n)
            relay = self.db.by_last_byte(h.relay_node) if h.relay_node else []
            hops = h.hop_start - h.hop_limit if h.hop_start else None
            nh = self.db.by_last_byte(h.next_hop) if h.next_hop else []
            lines += [("— Meshtastic", ""),
                      ("de", self.db.label(h.frm)),
                      ("para", self.db.label(h.to)),
                      ("id do pacote", f"0x{h.id:08x}" + (f"  (retransmissão — já visto em #{self.sdr_seen[key][0]})" if repeat else "")),
                      ("saltos", f"hop_limit {h.hop_limit}, hop_start {h.hop_start}"
                                 + (f" → já percorreu {hops} salto(s)" if hops is not None else "")),
                      ("flags", f"0x{h.flags:02x} want_ack={'sim' if h.want_ack else 'não'} via_mqtt={'sim' if h.via_mqtt else 'não'}"),
                      ("hash do canal", f"0x{h.channel:02x}"),
                      ("next_hop", f"0x{h.next_hop:02x}" + (f" ({', '.join(x.label() for x in nh)})" if nh else "")),
                      ("relay_node", f"0x{h.relay_node:02x}" + (f" ({', '.join(x.label() for x in relay)})" if relay else
                                                               (" (origem)" if h.relay_node == (h.frm & 0xFF) else "")))]
            enc = raw[16:]
            dec = decrypt(h, enc, self.ring, self.db)
            record["mesh"] = dict(de=node_id(h.frm), para=node_id(h.to), id=f"0x{h.id:08x}", hop_limit=h.hop_limit,
                                  hop_start=h.hop_start, want_ack=h.want_ack, via_mqtt=h.via_mqtt,
                                  canal_hash=h.channel, next_hop=h.next_hop, relay_node=h.relay_node, repeticao=repeat,
                                  cifrado_hex=enc.hex())
            if dec.data is not None:
                self.stats["decifrados"] += 1
                d = dec.data
                content = decode_content(d)
                title_port = content["porta"]
                style = "bold green" if title_port == "TEXT_MESSAGE_APP" else "cyan"
                lines += [("— Decifragem", ""), ("método", dec.method + (" (autenticado)" if dec.authenticated else ""))]
                meta = data_meta(d)
                lines += [("— Conteúdo", ""), ("porta", f"{content['porta']} ({content['porta_num']})")]
                lines += [(k, str(v)) for k, v in meta.items()]
                if "texto" in content:
                    lines.append(("TEXTO", content["texto"]))
                for k, v in flatten(content.get("campos", {}), self.db):
                    lines.append((k, str(v)))
                lines += enrich(content, self.db, h)
                if "hex" in content:
                    lines.append(("payload hex", content["hex"]))
                if "obs" in content:
                    lines.append(("obs", content["obs"]))
                if content["porta"] == "NODEINFO_APP" and "campos" in content:
                    self.db.update_user(h.frm, content["campos"], "ar (SDR)")
                record["decifragem"] = dec.method
                record["conteudo"] = content
                record["meta"] = meta
            else:
                self.stats["cifrados"] += 1
                title_port, style = "CIFRADO", "magenta"
                lines += [("— Decifragem", ""), ("status", dec.note), ("payload cifrado", f"{len(enc)} bytes")]
                record["decifragem"] = None
                record["nota"] = dec.note
            m = self.mod_seen.get(key)
            if self.module:
                if m:
                    lines += [("— Módulo", ""), ("também recebido", f"RSSI {m['rssi']} dBm, SNR {m['snr']} dB, "
                                                                   f"hop_limit {m['hop_limit']}")]
                    record["modulo"] = m
                elif h.frm == self.db.my_num:
                    lines += [("— Módulo", ""), ("origem", "transmitido pelo seu próprio nó")]
                if not repeat and not m and h.frm != self.db.my_num:
                    self.stats["so_sdr"] += 1
        elif crc_ok:
            lines.append(("obs", "quadro LoRa curto demais para ser Meshtastic"))

        if not crc_ok:
            title_port, style = "CRC FALHOU", "red"
        if a.hex or not crc_ok:
            lines.append(("bruto", raw.hex()))
        self.print_frame(n, fr, title_port, style, lines)
        if self.log:
            self.log.write(json.dumps(record, ensure_ascii=False, default=str) + "\n")
            self.log.flush()

    def print_frame(self, n, fr, title, style, lines):
        if not console:
            print(f"#{n} {self.frame_time(fr)} {title}")
            for k, v in lines:
                print(f"  {k}: {v}")
            return
        t = Text()
        w = max((len(k) for k, v in lines if v != ""), default=10)
        for k, v in lines:
            if v == "" and k.startswith("—"):
                t.append(f"{k[2:]}\n", style="bold underline")
                continue
            t.append(f"  {k:<{w}}  ", style="dim" if k != "TEXTO" else "bold")
            t.append(f"{v}\n", style="bold green" if k == "TEXTO" else "")
        console.print(Panel(t, title=f"#{n}  {self.frame_time(fr)}  {title}", title_align="left",
                            border_style=style, expand=False))

    def print_stats(self, final=False):
        s = self.stats
        rx = self.rx.stats
        msg = (f"[estatística] preâmbulos {rx['preambulos']} · quadros {s['quadros']} (CRC ok {s['crc_ok']}, "
               f"ruim {s['crc_ruim']}) · decifrados {s['decifrados']} · cifrados {s['cifrados']}")
        if self.module:
            msg += (f" · módulo recebeu {s['modulo_rx']} (ambos {s['modulo_e_sdr']}, só módulo {s['so_modulo']}, "
                    f"só SDR {s['so_sdr']})")
        if getattr(self.src, "dropped", 0):
            msg += f" · amostras perdidas {self.src.dropped}"
        say(msg, "bold" if final else "dim")

    # ------------------------------------------------------------ laço principal
    def run(self):
        a = self.a
        self.freq_hz = a.freq * 1e6
        rate, decim, offset = plan_rates(self.bw, a.taxa)
        os_ = 4 if decim % 4 == 0 else (2 if decim % 2 == 0 else 1)   # sobreamostragem mantida no receptor
        self.rx = LoRaReceiver(self.sf, self.bw, self.ldro, min_preamble=a.min_preambulo, detect_ratio=a.limiar,
                               os=os_, carrier_hz=self.freq_hz)
        if a.arquivo:
            self.src = FileSource(a.arquivo, a.formato)
            # num arquivo, o sinal está deslocado de --deslocamento em relação ao centro da gravação
            offset = a.deslocamento if a.deslocamento is not None else offset
            say(f"Lendo {a.arquivo} ({self.src.fmt}) a {rate / 1e6:g} MS/s, sinal em {offset / 1e3:+.0f} kHz do centro", "cyan")
        else:
            center = self.freq_hz - offset
            try:
                self.src = RtlSource(center, rate, a.ganho, a.ppm, a.dispositivo, a.bias_tee, a.serial_sdr, a.gravar)
            except Exception as e:  # noqa: BLE001
                say(f"Não consegui abrir o RTL-SDR: {e}", "bold red")
                say("  • Instale o driver WinUSB com o Zadig (Options > List All Devices > 'Bulk-In, Interface (Interface 0)').\n"
                    "  • Feche SDR#, SDR++, rtl_tcp ou outro programa que esteja usando o dongle.\n"
                    "  • RTL-SDR Blog V4: copie rtlsdr.dll e libusb-1.0.dll do driver da RTL-SDR Blog para a pasta drivers\\.\n"
                    "  • Mais de um dongle: use --dispositivo N ou --serial-sdr.", "yellow")
                if self.module:
                    self.module.close()
                sys.exit(1)
            if self.src.dll_dir:
                say(f"Usando rtlsdr.dll de {self.src.dll_dir}", "cyan")
            for w in self.src.warnings:
                say(f"[aviso] {w}", "bold yellow")
            info = self.src.info()
            say(f"RTL-SDR: centro {info['centro_hz'] / 1e6:.6f} MHz (sinal {self.freq_hz / 1e6:.6f} MHz a +{offset / 1e3:.0f} kHz), "
                f"{rate / 1e6:g} MS/s, ganho {info['ganho']}, ppm {info['ppm']}", "cyan")
            if a.gravar:
                say(f"Gravando IQ bruto em {a.gravar} (cu8, {rate:.0f} S/s, sinal a +{offset:.0f} Hz)", "cyan")
        self.fe = FrontEnd(rate, decim // os_, offset, self.bw)
        say(f"Escutando SF{self.sf} BW{self.bw / 1e3:g} kHz CR4/{self.cr + 4}. Ctrl+C para sair.\n", "bold cyan")
        self.t0_wall = time.time()
        try:
            for chunk in self.src.chunks():
                if chunk is not None:
                    for fr in self.rx.feed(self.fe.process(chunk)):
                        self.handle(fr)
                self.poll_module()
                if a.estatisticas and time.time() - self.last_stats > a.estatisticas:
                    self.print_stats()
                    self.last_stats = time.time()
            for fr in self.rx.flush():
                self.handle(fr)
        except KeyboardInterrupt:
            pass
        finally:
            self.poll_module()
            self.print_stats(final=True)
            self.src.close()
            if self.module:
                self.module.close()
            if self.log:
                self.log.close()


def main(argv=None):
    p = argparse.ArgumentParser(description="Decodificador Meshtastic com RTL-SDR + módulo USB",
                                formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    g = p.add_argument_group("rádio")
    g.add_argument("--freq", type=float, default=919.875, help="frequência do canal em MHz")
    g.add_argument("--preset", default="LongFast", help="preset Meshtastic (" + ", ".join(PRESETS) + ")")
    g.add_argument("--sf", type=int, help="força o spreading factor")
    g.add_argument("--bw", type=float, help="força a largura de banda em kHz")
    g.add_argument("--cr", type=int, help="força o coding rate (1=4/5 .. 4=4/8)")
    g.add_argument("--ldro", type=lambda s: s.lower() in ("1", "sim", "true", "on"), default=None,
                   help="força o low data rate optimize (padrão: automático, como o firmware)")
    g = p.add_argument_group("RTL-SDR")
    g.add_argument("--ganho", default="0", help="ganho em dB ou 'auto' (0 evita saturar com o módulo perto)")
    g.add_argument("--ppm", type=int, default=0, help="correção de frequência do cristal")
    g.add_argument("--dispositivo", type=int, default=0, help="índice do RTL-SDR")
    g.add_argument("--serial-sdr", help="escolhe o RTL-SDR pelo número de série")
    g.add_argument("--bias-tee", action="store_true", help="liga a alimentação da antena (RTL-SDR Blog)")
    g.add_argument("--taxa", type=float, help="taxa de amostragem (padrão: escolhida pelo BW)")
    g.add_argument("--gravar", help="grava o IQ bruto (.cu8) enquanto decodifica")
    g = p.add_argument_group("arquivo IQ (em vez do RTL-SDR)")
    g.add_argument("--arquivo", help="arquivo IQ: .cu8 (rtl_sdr), .cs8, .cs16, .cf32/.cfile")
    g.add_argument("--formato", choices=["cu8", "cs8", "cs16", "cf32"], help="formato do arquivo")
    g.add_argument("--deslocamento", type=float, help="posição do sinal em Hz em relação ao centro da gravação")
    g = p.add_argument_group("módulo e chaves")
    g.add_argument("--porta", help="porta serial do módulo (ex.: COM5); padrão: detecção automática")
    g.add_argument("--sem-modulo", action="store_true", help="não conecta ao módulo")
    g.add_argument("--canal", action="append", help="canal extra NOME:PSK[:aead] (PSK em base64 como no app)")
    g.add_argument("--chave-privada", help="chave privada PKI do seu nó em base64 (se não vier do módulo)")
    g.add_argument("--meu-no", help="id do seu nó (ex.: !a1b2c3d4), para decifrar DMs sem o módulo")
    g.add_argument("--espera-modulo", type=float, default=6.0, help="segundos antes de avisar 'só o módulo recebeu'")
    g = p.add_argument_group("saída")
    g.add_argument("--log", default="pacotes.jsonl", help="arquivo JSONL com tudo ('' desliga)")
    g.add_argument("--hex", action="store_true", help="mostra os bytes brutos de cada quadro")
    g.add_argument("--mostrar-falhas", action="store_true", help="mostra quadros com CRC ruim")
    g.add_argument("--estatisticas", type=float, default=60, help="intervalo da linha de estatística em s (0 desliga)")
    g = p.add_argument_group("detecção (avançado)")
    g.add_argument("--min-preambulo", type=int, default=6, help="upchirps seguidos para detectar preâmbulo")
    g.add_argument("--limiar", type=float, default=10.0, help="pico/média mínimo para considerar um chirp")
    g.add_argument("--listar-portas", action="store_true", help="lista as portas seriais e sai")
    g.add_argument("--diagnostico", action="store_true",
                   help="grava alguns segundos em banda larga, mostra rajadas de RF e testa a recepção")
    g.add_argument("--diag-segundos", type=float, default=20, help="duração do diagnóstico")
    a = p.parse_args(argv)
    if a.listar_portas:
        from module_link import list_serial_ports
        lst = list_serial_ports()
        print("\n".join(lst) if lst else "nenhuma porta serial encontrada")
        return
    a.arquivo_somente = False
    if a.arquivo and not a.taxa:
        p.error("com --arquivo informe --taxa (taxa de amostragem da gravação)")
    if a.log == "":
        a.log = None
    if a.diagnostico:
        a.sem_modulo = True
        if a.arquivo and not a.taxa:
            p.error("com --arquivo informe --taxa")
    app = App(a)
    app.setup()
    if a.diagnostico:
        from diagnostico import run_diag
        run_diag(a, a.freq * 1e6, app.sf, app.bw, app.ldro, say)
        return
    app.run()


if __name__ == "__main__":
    main()
