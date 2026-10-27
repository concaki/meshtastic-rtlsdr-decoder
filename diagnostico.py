# SPDX-License-Identifier: GPL-3.0-or-later
"""
Modo diagnóstico: grava alguns segundos com banda larga (2 MS/s, ±1 MHz em torno do canal),
procura rajadas de RF no espectro, mede frequência/largura/força de cada uma, verifica saturação
e roda o decodificador ao mesmo tempo. Salva a gravação para análise posterior.
"""
from __future__ import annotations

import time

import numpy as np

from lora_phy import LoRaReceiver
from sources import FileSource, FrontEnd, RtlSource, cu8_to_complex

NFFT = 4096


class BurstFinder:
    def __init__(self, fs: float, center_hz: float, frame_s: float = 0.05):
        self.fs, self.center = fs, center_hz
        self.frame = int(fs * frame_s)
        self.win = np.hanning(NFFT).astype(np.float32)
        self.frames: list[np.ndarray] = []
        self.pending = np.zeros(0, np.complex64)
        self.freqs = np.fft.fftshift(np.fft.fftfreq(NFFT, 1 / fs))

    def feed(self, x: np.ndarray):
        x = np.concatenate([self.pending, x])
        n = len(x) // self.frame
        for i in range(n):
            seg = x[i * self.frame:(i + 1) * self.frame]
            k = len(seg) // NFFT
            blk = seg[:k * NFFT].reshape(k, NFFT) * self.win
            p = np.mean(np.abs(np.fft.fft(blk, axis=1)) ** 2, axis=0)
            self.frames.append(10 * np.log10(np.fft.fftshift(p) + 1e-12))
        self.pending = x[n * self.frame:]

    def analyse(self, min_excess_db: float = 4.0):
        if not self.frames:
            return [], None, None
        S = np.array(self.frames)                     # tempo x frequência (dB)
        floor = np.median(S, axis=0)                  # piso de ruído por bin
        # suavizar em frequência (~60 kHz) para medir energia de sinais de 125-500 kHz
        k = max(1, int(60e3 / (self.fs / NFFT)))
        ker = np.ones(k) / k
        lin = 10 ** ((S - floor) / 10)
        sm = np.array([np.convolve(r, ker, mode="same") for r in lin])
        exc = 10 * np.log10(sm + 1e-12)
        edge = int(NFFT * 0.05)
        exc[:, :edge] = 0
        exc[:, -edge:] = 0
        dc = NFFT // 2
        exc[:, dc - 3:dc + 4] = 0                     # ignora o pico DC
        active = exc.max(axis=1) > min_excess_db
        bursts, i = [], 0
        dt = self.frame / self.fs
        while i < len(active):
            if not active[i]:
                i += 1
                continue
            j = i
            while j < len(active) and active[j]:
                j += 1
            # junta rajadas separadas por menos de 100 ms
            while j < len(active):
                nxt = j
                while nxt < len(active) and not active[nxt] and (nxt - j) * dt < 0.1:
                    nxt += 1
                if nxt < len(active) and active[nxt] and (nxt - j) * dt < 0.1:
                    j = nxt
                    while j < len(active) and active[j]:
                        j += 1
                else:
                    break
            seg = exc[i:j].mean(axis=0)
            pk = int(np.argmax(seg))
            thr = max(seg[pk] - 6.0, 2.0)
            lo = hi = pk                      # largura contígua acima do limiar em torno do pico
            while lo > 0 and seg[lo - 1] > thr:
                lo -= 1
            while hi < len(seg) - 1 and seg[hi + 1] > thr:
                hi += 1
            w = 10 ** (seg[lo:hi + 1] / 10)
            fc = float(np.sum(self.freqs[lo:hi + 1] * w) / np.sum(w))
            bursts.append(dict(t=i * dt, dur=(j - i) * dt, f_hz=self.center + fc,
                               bw_hz=(hi - lo + 1) * self.fs / NFFT, excess_db=float(seg[pk])))
            i = j
        return bursts, floor, S


def run_diag(a, freq_hz: float, sf: int, bw: float, ldro: bool, say):
    rate = 2_000_000
    offset = 500e3
    say(f"\n=== DIAGNÓSTICO ({a.diag_segundos:g} s) ===", "bold cyan")
    clip_count = total = 0
    if a.arquivo:
        rate = a.taxa
        offset = a.deslocamento if a.deslocamento is not None else rate / 4
        src = FileSource(a.arquivo, a.formato)
        center = freq_hz - offset
        say(f"Arquivo {a.arquivo}, {rate / 1e6:g} MS/s", "cyan")
    else:
        center = freq_hz - offset
        rec = a.gravar or "diagnostico.cu8"
        src = RtlSource(center, rate, a.ganho, a.ppm, a.dispositivo, a.bias_tee, a.serial_sdr, rec)
        info = src.info()
        tuners = {1: "E4000", 2: "FC0012", 3: "FC0013", 4: "FC2580", 5: "R820T/R820T2", 6: "R828D"}
        say(f"Tuner: {tuners.get(src.tuner, src.tuner)} · DLL: {src.dll_dir or 'embutida no pyrtlsdr'}", "cyan")
        say(f"Ganho pedido {a.ganho} → informado pelo dongle {info['ganho']} dB · ganhos válidos: "
            f"{', '.join(f'{g:g}' for g in info['ganhos_validos']) or 'nenhum informado'}", "cyan")
        say(f"Centro {center / 1e6:.6f} MHz, observando {(center - rate / 2) / 1e6:.3f}–{(center + rate / 2) / 1e6:.3f} MHz", "cyan")
        say(f"Gravando em {rec}", "cyan")
        for w in src.warnings:
            say(f"[aviso] {w}", "bold yellow")
        for w in getattr(src, "notes", []):
            say(f"[obs] {w}", "cyan")
        say("\n>>> AGORA mande 2 ou 3 mensagens pelo seu módulo (app no celular) durante a captura <<<\n", "bold yellow")

    decim = int(round(rate / bw))
    os_ = 4 if decim % 4 == 0 else (2 if decim % 2 == 0 else 1)
    fe = FrontEnd(rate, decim // os_, offset, bw)
    rx = LoRaReceiver(sf, bw, ldro, os=os_, carrier_hz=freq_hz)
    bf = BurstFinder(rate, center)
    frames = []
    t0 = time.time()
    need = int(a.diag_segundos * rate)
    got = 0
    last = -1
    try:
        for chunk in src.chunks():
            if chunk is None:
                continue
            got += len(chunk)
            bf.feed(chunk)
            mag = np.maximum(np.abs(chunk.real), np.abs(chunk.imag))
            clip_count += int(np.sum(mag > 0.98))
            total += len(chunk)
            frames += rx.feed(fe.process(chunk))
            sec = int(got / rate)
            if sec != last and not a.arquivo:
                last = sec
                print(f"\r  capturando... {sec:3d}/{a.diag_segundos:g} s   quadros LoRa: {len(frames)}", end="", flush=True)
            if not a.arquivo and got >= need:
                break
    except KeyboardInterrupt:
        pass
    finally:
        frames += rx.flush()
        src.close()
    print()

    bursts, floor, S = bf.analyse()
    clip = clip_count / max(total, 1) * 100
    say(f"\nSaturação do ADC: {clip:.3f}% das amostras" + ("  ← ALTA, baixe o ganho" if clip > 0.5 else "  (ok)"),
        "bold red" if clip > 0.5 else "green")
    if floor is not None:
        say(f"Piso de ruído: {np.median(floor):.1f} dB (relativo) · variação na banda {np.ptp(np.percentile(floor, [5, 95])):.1f} dB")

    say(f"\nRajadas de RF encontradas: {len(bursts)}", "bold")
    near = []
    for b in bursts:
        df = b["f_hz"] - freq_hz
        tag = ""
        if abs(df) < bw and b["bw_hz"] > bw * 0.4:
            tag = "  ← provável Meshtastic no canal"
            near.append(b)
        say(f"  t={b['t']:6.2f}s  dur {b['dur'] * 1000:5.0f} ms  {b['f_hz'] / 1e6:.4f} MHz ({df / 1e3:+7.1f} kHz)  "
            f"largura ~{b['bw_hz'] / 1e3:4.0f} kHz  +{b['excess_db']:4.1f} dB{tag}")
    say(f"\nQuadros LoRa decodificados: {len(frames)} (CRC ok {sum(1 for f in frames if f.payload and f.payload.crc_ok)})", "bold")
    for f in frames:
        ok = f.payload and f.payload.crc_ok
        say(f"  {f.header.length} bytes, CRC {'OK' if ok else 'ruim'}, desvio {f.cfo_hz:+.0f} Hz, SNR {f.snr_db:.1f} dB"
            + f", deriva {f.drift_bins * bw / (1 << sf):+.1f} Hz/símbolo"
            + (f", {f.payload.corrected_bits} bits corrigidos" if f.payload else ""),
            "green" if ok else "yellow")

    say("\nConclusão:", "bold")
    if frames and any(f.payload and f.payload.crc_ok for f in frames):
        cfo = np.median([f.cfo_hz for f in frames])
        ppm = cfo / freq_hz * 1e6
        say(f"  O SDR está recebendo e decodificando. Desvio médio {cfo:+.0f} Hz ({ppm:+.1f} ppm).", "green")
        if abs(ppm) > 20:
            say(f"  Sugestão: rode com --ppm {int(round(-ppm))} para centralizar.", "green")
    elif near:
        say("  Há rajadas no canal, mas nada decodificou. Se houve saturação, baixe o ganho; senão tente 10-20 dB. Confira se o",
            "yellow")
        say("  módulo usa LongFast. Me mande o arquivo diagnostico.cu8 que eu analiso.", "yellow")
    elif bursts:
        b = max(bursts, key=lambda x: x["excess_db"])
        say(f"  Rajadas apareceram, mas fora do canal (mais forte em {b['f_hz'] / 1e6:.4f} MHz). Se coincidiram com", "yellow")
        say("  as mensagens enviadas, o dongle está sintonizando com erro: provável cristal diferente do esperado", "yellow")
        say("  pela DLL. Me mande o print e o arquivo diagnostico.cu8.", "yellow")
    else:
        say("  Nenhuma rajada visível. Possíveis causas: antena (a que vem com o dongle é para TV, ~500 MHz),", "red")
        say("  ganho não aplicado, módulo transmitindo em outra frequência/região, ou o dongle sintonizando em", "red")
        say("  outro lugar (cristal do R828D). Teste com o módulo bem perto do SDR e me mande o print.", "red")
