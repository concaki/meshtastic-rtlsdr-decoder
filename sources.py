# SPDX-License-Identifier: GPL-3.0-or-later
"""Fontes de IQ (RTL-SDR ao vivo ou arquivo) e front-end de mistura/filtro/decimação."""
from __future__ import annotations

import math
import queue
import threading
import time

import numpy as np
from scipy.signal import firwin, oaconvolve


def plan_rates(bw: float, rate: float | None = None):
    """Escolhe taxa do RTL-SDR (múltiplo inteiro do BW) e o deslocamento do sinal em relação ao centro."""
    if rate is None:
        d = max(4, math.ceil(0.9e6 / bw))
        rate = bw * d
    d = rate / bw
    if abs(d - round(d)) > 1e-6:
        raise ValueError(f"a taxa {rate:.0f} precisa ser múltiplo inteiro do BW {bw:.0f}")
    d = int(round(d))
    if not (900_001 <= rate <= 3_200_000 or 225_001 <= rate <= 300_000):
        raise ValueError(f"taxa {rate:.0f} fora das faixas aceitas pelo RTL-SDR")
    offset = rate / 4.0  # sinal fica a +fs/4 do centro, longe do pico DC
    return rate, d, offset


class FrontEnd:
    """Mistura o sinal para banda base, filtra e decima até fs = BW."""

    def __init__(self, fs_in: float, decim: int, offset_hz: float, bw: float, taps: int = 129,
                 max_cfo_hz: float = 90e3):
        self.fs, self.decim, self.offset = fs_in, decim, offset_hz
        # passa ±(BW/2 + desvio máximo), limitado pela taxa de saída; o filtro fino é feito depois
        fs_out = fs_in / decim
        cutoff = min(0.5 * bw + max_cfo_hz, 0.45 * fs_out)
        self.h = firwin(taps, cutoff, fs=fs_in).astype(np.float32)
        self.tail = np.zeros(taps - 1, np.complex64)
        self.n = 0            # amostras de entrada já processadas
        self.phase = 0        # fase de decimação
        self.out_count = 0

    def process(self, x: np.ndarray) -> np.ndarray:
        n0 = self.n
        idx = np.arange(n0, n0 + len(x), dtype=np.float64)
        ph = (self.offset / self.fs) * idx
        ph -= np.floor(ph)
        y = x.astype(np.complex64) * np.exp(-2j * np.pi * ph).astype(np.complex64)
        self.n += len(x)
        z = np.concatenate([self.tail, y])
        self.tail = z[-(len(self.h) - 1):]
        f = oaconvolve(z, self.h, mode="valid").astype(np.complex64)
        out = f[self.phase::self.decim]
        self.phase = (self.phase - len(f)) % self.decim
        self.out_count += len(out)
        return out


def cu8_to_complex(raw: bytes | np.ndarray) -> np.ndarray:
    a = np.frombuffer(raw, np.uint8).astype(np.float32)
    a = (a - 127.4) / 128.0
    return (a[0::2] + 1j * a[1::2]).astype(np.complex64)


class FileSource:
    FORMATS = {".cu8": "cu8", ".bin": "cu8", ".raw": "cu8", ".cs8": "cs8",
               ".cs16": "cs16", ".cf32": "cf32", ".cfile": "cf32", ".fc32": "cf32", ".iq": "cf32"}

    def __init__(self, path: str, fmt: str | None = None, chunk: int = 262144):
        import os
        self.path = path
        ext = os.path.splitext(path)[1].lower()
        self.fmt = fmt or self.FORMATS.get(ext, "cu8")
        self.chunk = chunk
        self.f = open(path, "rb")
        self.dropped = 0

    def chunks(self):
        bps = {"cu8": 2, "cs8": 2, "cs16": 4, "cf32": 8}[self.fmt]
        while True:
            raw = self.f.read(self.chunk * bps)
            if not raw:
                return
            raw = raw[: len(raw) - len(raw) % bps]
            if self.fmt == "cu8":
                yield cu8_to_complex(raw)
            elif self.fmt == "cs8":
                a = np.frombuffer(raw, np.int8).astype(np.float32) / 128.0
                yield (a[0::2] + 1j * a[1::2]).astype(np.complex64)
            elif self.fmt == "cs16":
                a = np.frombuffer(raw, np.int16).astype(np.float32) / 32768.0
                yield (a[0::2] + 1j * a[1::2]).astype(np.complex64)
            else:
                yield np.frombuffer(raw, np.complex64).copy()

    def close(self):
        self.f.close()


def _prepare_rtlsdr_dll():
    """No Windows, prefere rtlsdr.dll da pasta drivers/ (ex.: driver do RTL-SDR Blog V4)."""
    import os
    import sys
    if sys.platform != "win32":
        return None
    here = os.path.dirname(os.path.abspath(__file__))
    for d in (os.path.join(here, "drivers"), here):
        if os.path.exists(os.path.join(d, "rtlsdr.dll")):
            os.add_dll_directory(d)
            os.environ["PATH"] = d + os.pathsep + os.environ.get("PATH", "")
            sys.modules["pyrtlsdrlib"] = None  # não usar a DLL embutida do pacote
            return d
    return None


class RtlSource:
    """Lê o RTL-SDR numa thread separada para não perder amostras."""

    def __init__(self, center_hz: float, rate: float, gain="auto", ppm: int = 0, index: int = 0,
                 bias_tee: bool = False, serial: str | None = None, record_path: str | None = None,
                 chunk: int = 65536):
        self.dll_dir = _prepare_rtlsdr_dll()
        from rtlsdr import RtlSdr
        if serial:
            index = RtlSdr.get_device_index_by_serial(serial)
        self.sdr = RtlSdr(device_index=index)
        self.sdr.sample_rate = rate
        self.sdr.center_freq = center_hz
        if ppm:
            self.sdr.freq_correction = int(ppm)
        self.warnings = []
        try:
            self.tuner = int(self.sdr.get_tuner_type())   # 5 = R820T, 6 = R828D (RTL-SDR Blog V4)
        except Exception:  # noqa: BLE001
            self.tuner = -1
        self.notes = []
        if self.tuner == 6 and not self.dll_dir:
            self.notes.append(
                "tuner R828D (RTL-SDR Blog V4 ou clone genérico). Clones funcionam com a DLL padrão; "
                "só no V4 original vale usar a DLL da RTL-SDR Blog em drivers\\.")
        if isinstance(gain, str) and gain.lower() == "auto":
            self.sdr.gain = "auto"
        else:
            want = float(gain)
            try:
                valid = [g / 10 for g in self.sdr.get_gains()]
            except Exception:  # noqa: BLE001
                valid = []
            if valid:
                want = min(valid, key=lambda g: abs(g - want))
            self.sdr.set_manual_gain_enabled(True)
            self.sdr.gain = want
            got = self.sdr.get_gain()
            if valid and abs(got - want) > 1:
                self.warnings.append(f"pedi ganho {want} dB mas o dongle informa {got} dB "
                                     f"(ganhos válidos: {', '.join(f'{g:g}' for g in valid) or 'nenhum'})")
            elif not valid:
                self.warnings.append("o driver não informou a lista de ganhos válidos; o ganho pode não ter sido aplicado")
        if bias_tee:
            try:
                self.sdr.set_bias_tee(True)
            except Exception as e:  # noqa: BLE001
                print(f"[aviso] bias-tee não suportado: {e}")
        self.chunk = chunk
        # fila grande: o decodificador pode atrasar alguns segundos sem perder amostras
        self.q: queue.Queue = queue.Queue(maxsize=max(64, int(20 * rate / chunk)))
        self.dropped = 0
        self.stop_evt = threading.Event()
        self.rec = open(record_path, "wb") if record_path else None
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def info(self) -> dict:
        s = self.sdr
        try:
            gains = [g / 10 for g in s.get_gains()]
        except Exception:  # noqa: BLE001
            gains = []
        return {"centro_hz": s.center_freq, "taxa": s.sample_rate, "ganho": s.gain,
                "ppm": s.freq_correction, "ganhos_validos": gains}

    def _cb(self, values, _ctx):
        # leitura assíncrona: a libusb mantém vários buffers em voo, então não há buracos entre
        # leituras (a leitura síncrona perdia ~0,4 ms a cada bloco e corrompia pacotes)
        raw = bytes(values)
        if self.rec:
            self.rec.write(raw)
        try:
            self.q.put_nowait((time.time(), raw))
        except queue.Full:
            self.dropped += len(raw) // 2

    def _run(self):
        try:
            self.sdr.DEFAULT_ASYNC_BUF_NUMBER = 32
            self.sdr.read_bytes_async(self._cb, num_bytes=2 * self.chunk)
        except Exception as e:  # noqa: BLE001
            if not self.stop_evt.is_set():
                self.q.put(e)

    def chunks(self):
        while not self.stop_evt.is_set():
            try:
                item = self.q.get(timeout=0.5)
            except queue.Empty:
                yield None
                continue
            if isinstance(item, Exception):
                raise item
            _, raw = item
            yield cu8_to_complex(raw)

    def close(self):
        self.stop_evt.set()
        try:
            self.sdr.cancel_read_async()
        except Exception:  # noqa: BLE001
            pass
        try:
            self.thread.join(timeout=3)
        except Exception:  # noqa: BLE001
            pass
        try:
            self.sdr.close()
        finally:
            if self.rec:
                self.rec.close()
