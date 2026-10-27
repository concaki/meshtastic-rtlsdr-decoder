#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""
Gera uma gravação .cu8 sintética (como a do rtl_sdr) com vários pacotes Meshtastic, para testar o
decodificador sem rádio. Sinal LongFast a +250 kHz do centro, 1 MS/s.

  python tools/gerar_sinal_teste.py teste.cu8
  python mesh_sdr.py --arquivo teste.cu8 --taxa 1000000 --sem-modulo --canal "Familia:<psk>" ...
(o script imprime a linha de comando completa no final)
"""
from __future__ import annotations

import base64
import json
import os
import struct
import sys

import numpy as np
from cryptography.hazmat.primitives.asymmetric.x25519 import X25519PrivateKey
from cryptography.hazmat.primitives.ciphers.aead import AESCCM
from cryptography.hazmat.primitives.serialization import Encoding, PublicFormat, PrivateFormat, NoEncryption
from scipy.signal import resample_poly

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from lora_phy import modulate  # noqa: E402
from mesh_decode import (DEFAULT_PSK, _nonce, aes_ctr, channel_hash, pki_shared_key)  # noqa: E402
from meshtastic.protobuf import mesh_pb2, portnums_pb2, telemetry_pb2  # noqa: E402

SF, BW, CR = 11, 250e3, 1
FS = 1_000_000
OFFSET = 250e3

A, B, C = 0xA1B2C3D4, 0x0BADF00D, 0x12345678
rng = np.random.default_rng(7)


def keypair():
    sk = X25519PrivateKey.generate()
    priv = sk.private_bytes(Encoding.Raw, PrivateFormat.Raw, NoEncryption())
    pub = sk.public_key().public_bytes(Encoding.Raw, PublicFormat.Raw)
    return priv, pub


A_PRIV, A_PUB = keypair()
B_PRIV, B_PUB = keypair()
FAM_PSK = bytes(rng.integers(0, 256, 32, dtype=np.uint8))
SEG_PSK = bytes(rng.integers(0, 256, 16, dtype=np.uint8))


def header(to, frm, pid, hop_limit, hop_start, chash, want_ack=False, relay=None, next_hop=0):
    flags = (hop_limit & 7) | (0x08 if want_ack else 0) | ((hop_start & 7) << 5)
    return struct.pack("<III", to, frm, pid) + bytes([flags, chash, next_hop, (relay if relay is not None else frm) & 0xFF])


def data(port, payload: bytes, **kw) -> bytes:
    d = mesh_pb2.Data(portnum=port, payload=payload, **kw)
    d.bitfield = 1
    return d.SerializeToString()


def chan_pkt(to, frm, pid, plain, name="LongFast", key=DEFAULT_PSK, hop=3, start=3, relay=None, want_ack=False):
    return header(to, frm, pid, hop, start, channel_hash(name, key), want_ack, relay) + aes_ctr(key, frm, pid, plain)


def aead_pkt(to, frm, pid, plain, name, key):
    h = header(to, frm, pid, 3, 3, channel_hash(name, key, True))
    aad = struct.pack("<II", frm, to)
    return h + AESCCM(key, tag_length=12).encrypt(_nonce(frm, pid)[:13], plain, aad)


def pki_pkt(to, frm, pid, plain, priv, peer_pub):
    extra = 0x5EED1234
    key = pki_shared_key(priv, peer_pub)
    ct = AESCCM(key, tag_length=8).encrypt(_nonce(frm, pid, extra)[:13], plain, None)
    return header(to, frm, pid, 3, 3, 0, True) + ct + struct.pack("<I", extra)


def build():
    P = portnums_pb2.PortNum
    pkts = []
    pkts.append(("texto LongFast", chan_pkt(0xFFFFFFFF, A, 0x1001, data(P.TEXT_MESSAGE_APP, "Olá da serra gaúcha! 📡".encode())), 12))
    u = mesh_pb2.User(id=f"!{B:08x}", long_name="Base Guaíba", short_name="GUA", hw_model=mesh_pb2.HardwareModel.HELTEC_V3,
                      public_key=B_PUB)
    pkts.append(("nodeinfo B", chan_pkt(0xFFFFFFFF, B, 0x2001, data(P.NODEINFO_APP, u.SerializeToString())), 8))
    ua = mesh_pb2.User(id=f"!{A:08x}", long_name="Movel Teste", short_name="MOV", hw_model=mesh_pb2.HardwareModel.TBEAM,
                       public_key=A_PUB)
    pkts.append(("nodeinfo A", chan_pkt(0xFFFFFFFF, A, 0x1002, data(P.NODEINFO_APP, ua.SerializeToString())), 6))
    pos = mesh_pb2.Position(latitude_i=int(-30.0346 * 1e7), longitude_i=int(-51.2177 * 1e7), altitude=12,
                            time=1793051640, sats_in_view=9, precision_bits=32)
    pkts.append(("posição A", chan_pkt(0xFFFFFFFF, A, 0x1003, data(P.POSITION_APP, pos.SerializeToString())), 4))
    tel = telemetry_pb2.Telemetry(time=1793051700)
    tel.device_metrics.battery_level = 87
    tel.device_metrics.voltage = 4.01
    tel.device_metrics.channel_utilization = 7.25
    tel.device_metrics.air_util_tx = 1.1
    tel.device_metrics.uptime_seconds = 93784
    pkts.append(("telemetria B", chan_pkt(0xFFFFFFFF, B, 0x2002, data(P.TELEMETRY_APP, tel.SerializeToString())), 0))
    pkts.append(("canal privado", chan_pkt(0xFFFFFFFF, C, 0x3001, data(P.TEXT_MESSAGE_APP, b"mensagem no canal Familia"),
                                           "Familia", FAM_PSK), 5))
    pkts.append(("DM PKI B->A", pki_pkt(A, B, 0x2003, data(P.TEXT_MESSAGE_APP, "DM secreta para você".encode()), B_PRIV, A_PUB), 3))
    pkts.append(("retransmissão do texto", chan_pkt(0xFFFFFFFF, A, 0x1001, data(P.TEXT_MESSAGE_APP, "Olá da serra gaúcha! 📡".encode()),
                                                    hop=2, start=3, relay=B), -4))
    rt = mesh_pb2.RouteDiscovery(route=[B], snr_towards=[24, 31])
    pkts.append(("traceroute", chan_pkt(A, C, 0x3002, data(P.TRACEROUTE_APP, rt.SerializeToString(), request_id=0x1004)), 2))
    ack = mesh_pb2.Routing(error_reason=mesh_pb2.Routing.Error.NONE)
    pkts.append(("ACK", chan_pkt(B, A, 0x1005, data(P.ROUTING_APP, ack.SerializeToString(), request_id=0x2003)), 3))
    pkts.append(("canal AEAD", aead_pkt(0xFFFFFFFF, C, 0x3003, data(P.TEXT_MESSAGE_APP, b"canal com AES-CCM"), "Seguro", SEG_PSK), 4))
    pkts.append(("desconhecido", chan_pkt(0xFFFFFFFF, C, 0x3004, data(P.TEXT_MESSAGE_APP, b"ninguem le"), "Outro", bytes(16)), 1))
    pkts.append(("texto fraco (-10 dB)", chan_pkt(0xFFFFFFFF, C, 0x3005, data(P.TEXT_MESSAGE_APP, b"sinal bem fraco"), ), -10))
    return pkts


def main(path="teste.cu8"):
    pkts = build()
    noise_amp = 0.02
    gap = int(0.25 * FS)
    out = [np.zeros(gap, np.complex64)]
    for name, raw, snr in pkts:
        x = modulate(raw, SF, CR)
        x = resample_poly(x, 4, 1).astype(np.complex64)
        x = x[int(rng.integers(0, 4)):]                       # atraso fracionário
        sig_amp = noise_amp * 0.5 * 10 ** (snr / 20)         # SNR medido na banda de 250 kHz (ruído/4)
        out.append(sig_amp * x * np.exp(1j * rng.uniform(0, 2 * np.pi)))
        out.append(np.zeros(gap + int(rng.integers(0, 50000)), np.complex64))
    s = np.concatenate(out)
    n = np.arange(len(s))
    cfo = 8_300.0                                            # erro do cristal ~9 ppm
    sfo = 1 + 9e-6                                           # mesmo erro no relógio de amostragem
    t = n * sfo
    s = np.interp(t, n, s.real) + 1j * np.interp(t, n, s.imag)
    s = s * np.exp(2j * np.pi * (OFFSET + cfo) * n / FS)
    s += noise_amp * (rng.standard_normal(len(s)) + 1j * rng.standard_normal(len(s))) / np.sqrt(2)
    s += 0.01  # pico DC do RTL
    iq = np.empty(2 * len(s), np.float32)
    iq[0::2], iq[1::2] = s.real, s.imag
    cu8 = np.clip(np.round(iq * 128 + 127.4), 0, 255).astype(np.uint8)
    cu8.tofile(path)
    b64 = lambda b: base64.b64encode(b).decode()  # noqa: E731
    print(f"{path}: {len(s) / FS:.1f} s, {len(pkts)} pacotes")
    for name, raw, snr in pkts:
        print(f"  - {name} ({len(raw)} bytes, SNR {snr} dB)")
    args = ["--arquivo", path, "--taxa", "1000000", "--sem-modulo", "--log", "teste.jsonl",
            "--canal", f"Familia:{b64(FAM_PSK)}", "--canal", f"Seguro:{b64(SEG_PSK)}:aead",
            "--chave-privada", b64(A_PRIV), "--meu-no", f"!{A:08x}"]
    # argumentos também em JSON (usado pelos testes; evita problemas de aspas/barras no Windows)
    with open(path + ".args.json", "w", encoding="utf-8") as f:
        json.dump(args, f)
    print("\nPara decodificar:")
    print(f'python mesh_sdr.py --arquivo {path} --taxa 1000000 --sem-modulo --log teste.jsonl '
          f'--canal "Familia:{b64(FAM_PSK)}" --canal "Seguro:{b64(SEG_PSK)}:aead" '
          f'--chave-privada {b64(A_PRIV)} --meu-no !{A:08x}')


if __name__ == "__main__":
    main(*sys.argv[1:])
