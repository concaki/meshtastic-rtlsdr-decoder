# SPDX-License-Identifier: GPL-3.0-or-later
"""
Camada física LoRa em Python puro (numpy).

Convenções conferidas com o gr-lora_sdr (EPFL) e o LoRaPHY:
  - chirp de referência u[n] = exp(j*pi*n^2/N - j*pi*n), símbolo s = u[n]*exp(j*2*pi*s*n/N)
  - TX: gray inverso + 1 (bloco de cabeçalho / LDRO: valor*4 + 1)
  - interleaver diagonal: inter[i][j] = cw[(i-j-1) mod sf_app] bit i
  - Hamming 4/5..4/8, whitening LFSR (x^8+x^6+x^5+x^4+1), CRC16 CCITT com XOR dos 2 últimos bytes
  - quadro: preâmbulo, 2 símbolos de sync word, 2,25 downchirps, cabeçalho explícito (8 símbolos, CR 4/8)
"""
from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, field

import numpy as np

# ----------------------------------------------------------------------------
# Tabelas e utilidades
# ----------------------------------------------------------------------------


def _whitening_seq(n: int = 255) -> np.ndarray:
    s, out = 0xFF, []
    for _ in range(n):
        out.append(s)
        b = ((s >> 7) ^ (s >> 5) ^ (s >> 4) ^ (s >> 3)) & 1
        s = ((s << 1) & 0xFE) | b
    return np.array(out, dtype=np.uint8)


WHITENING = _whitening_seq()


def crc16_ccitt(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def lora_payload_crc(payload: bytes) -> int:
    """CRC do payload LoRa como o chip Semtech calcula."""
    if len(payload) < 2:
        return crc16_ccitt(payload)
    crc = crc16_ccitt(payload[:-2])
    return crc ^ payload[-1] ^ (payload[-2] << 8)


def header_checksum(n0: int, n1: int, n2: int) -> int:
    b = lambda x, k: (x >> k) & 1  # noqa: E731
    c4 = b(n0, 3) ^ b(n0, 2) ^ b(n0, 1) ^ b(n0, 0)
    c3 = b(n0, 3) ^ b(n1, 3) ^ b(n1, 2) ^ b(n1, 1) ^ b(n2, 0)
    c2 = b(n0, 2) ^ b(n1, 3) ^ b(n1, 0) ^ b(n2, 3) ^ b(n2, 1)
    c1 = b(n0, 1) ^ b(n1, 2) ^ b(n1, 0) ^ b(n2, 2) ^ b(n2, 1) ^ b(n2, 0)
    c0 = b(n0, 0) ^ b(n1, 1) ^ b(n2, 3) ^ b(n2, 2) ^ b(n2, 1) ^ b(n2, 0)
    return (c4 << 4) | (c3 << 3) | (c2 << 2) | (c1 << 1) | c0


def hamming_encode(nib: int, cr: int) -> int:
    """cr = 1..4 (4/5 .. 4/8). Retorna palavra de 4+cr bits (MSB primeiro = d0)."""
    d0, d1, d2, d3 = (nib >> 0) & 1, (nib >> 1) & 1, (nib >> 2) & 1, (nib >> 3) & 1
    if cr == 1:
        bits = [d0, d1, d2, d3, d0 ^ d1 ^ d2 ^ d3]
    else:
        p0 = d0 ^ d1 ^ d2
        p1 = d1 ^ d2 ^ d3
        p2 = d0 ^ d1 ^ d3
        p3 = d0 ^ d2 ^ d3
        bits = [d0, d1, d2, d3, p0, p1, p2, p3][: 4 + cr]
    v = 0
    for bit in bits:
        v = (v << 1) | bit
    return v


_HAMMING_TABLE = {cr: np.array([hamming_encode(n, cr) for n in range(16)]) for cr in (1, 2, 3, 4)}
_POPCNT = np.array([bin(i).count("1") for i in range(256)])


def hamming_decode(cw: int, cr: int) -> tuple[int, int]:
    """Decodifica por distância mínima. Retorna (nibble, nº de bits divergentes)."""
    d = _POPCNT[_HAMMING_TABLE[cr] ^ cw]
    nib = int(np.argmin(d))
    return nib, int(d[nib])


def gray(x: int) -> int:
    return x ^ (x >> 1)


def inv_gray(x: int) -> int:
    out, m = x, x >> 1
    while m:
        out ^= m
        m >>= 1
    return out


def interleave(codewords: list[int], sf_app: int, cw_len: int) -> list[int]:
    """sf_app palavras de cw_len bits -> cw_len valores de sf_app bits."""
    out = []
    for i in range(cw_len):
        v = 0
        for j in range(sf_app):
            cw = codewords[(i - j - 1) % sf_app]
            bit = (cw >> (cw_len - 1 - i)) & 1
            v = (v << 1) | bit
        out.append(v)
    return out


def deinterleave(values: list[int], sf_app: int, cw_len: int) -> list[int]:
    cws = [0] * sf_app
    for i in range(cw_len):
        for j in range(sf_app):
            bit = (values[i] >> (sf_app - 1 - j)) & 1
            if bit:
                cws[(i - j - 1) % sf_app] |= 1 << (cw_len - 1 - i)
    return cws


def n_symbols(payload_len: int, sf: int, cr: int, crc: bool, ldro: bool, implicit: bool = False) -> int:
    num = 8 * payload_len - 4 * sf + 28 + 16 * int(crc) - 20 * int(implicit)
    den = 4 * (sf - 2 * int(ldro))
    return 8 + max(math.ceil(num / den) * (cr + 4), 0)


def time_on_air_ms(payload_len: int, sf: int, bw: float, cr: int, preamble: int = 16, crc=True, ldro=False) -> float:
    ts = (1 << sf) / bw
    return ((preamble + 4.25) + n_symbols(payload_len, sf, cr, crc, ldro)) * ts * 1000


# ----------------------------------------------------------------------------
# Codificação (usada pelo gerador de sinal de teste)
# ----------------------------------------------------------------------------


def encode_symbols(payload: bytes, sf: int, cr: int, crc: bool = True, ldro: bool = False) -> list[int]:
    """Bytes -> valores de bin (0..N-1) dos símbolos de dados, já com gray e +1."""
    N = 1 << sf
    nibbles: list[int] = []
    for i, b in enumerate(payload):
        w = b ^ int(WHITENING[i])
        nibbles += [w & 0xF, w >> 4]
    if crc:
        c = lora_payload_crc(payload)
        nibbles += [c & 0xF, (c >> 4) & 0xF, (c >> 8) & 0xF, (c >> 12) & 0xF]
    L = len(payload)
    h = [L >> 4, L & 0xF, (cr << 1) | int(crc)]
    chk = header_checksum(*h)
    h += [chk >> 4, chk & 0xF]
    allnib = h + nibbles

    symbols: list[int] = []
    # bloco do cabeçalho
    sf_app = sf - 2
    blk = (allnib[:sf_app] + [0] * sf_app)[:sf_app]
    rest = allnib[sf_app:]
    vals = interleave([hamming_encode(n, 4) for n in blk], sf_app, 8)
    symbols += [(inv_gray(v) * 4 + 1) % N for v in vals]
    sf_app = sf - 2 * int(ldro)
    cw_len = 4 + cr
    while rest:
        blk = (rest[:sf_app] + [0] * sf_app)[:sf_app]
        rest = rest[sf_app:]
        vals = interleave([hamming_encode(n, cr) for n in blk], sf_app, cw_len)
        if ldro:
            symbols += [(inv_gray(v) * 4 + 1) % N for v in vals]
        else:
            symbols += [(inv_gray(v) + 1) % N for v in vals]
    return symbols


def chirps(sf: int):
    N = 1 << sf
    n = np.arange(N)
    up = np.exp(1j * (np.pi * n * n / N - np.pi * n)).astype(np.complex64)
    return up, np.conj(up)


def modulate(payload: bytes, sf: int, cr: int, preamble: int = 16, sync_word: int = 0x2B,
             crc: bool = True, ldro: bool = False) -> np.ndarray:
    """Gera IQ complexo a 1 amostra/chip (fs = BW)."""
    N = 1 << sf
    up, down = chirps(sf)
    n = np.arange(N)

    def sym(s):
        return up * np.exp(1j * 2 * np.pi * s * n / N).astype(np.complex64)

    parts = [up] * preamble
    parts += [sym(((sync_word & 0xF0) >> 4) << 3), sym((sync_word & 0x0F) << 3)]
    parts += [down, down, down[: N // 4]]
    parts += [sym(s) for s in encode_symbols(payload, sf, cr, crc, ldro)]
    return np.concatenate(parts).astype(np.complex64)


# ----------------------------------------------------------------------------
# Decodificação a partir dos bins
# ----------------------------------------------------------------------------


@dataclass
class LoRaHeader:
    length: int
    cr: int
    has_crc: bool
    checksum_ok: bool
    extra_nibbles: list[int]
    corrected_bits: int


def header_symbols(hdr: "LoRaHeader", sf: int) -> list[int]:
    """Re-codifica o bloco do cabeçalho decodificado, devolvendo os 8 bins exatos transmitidos."""
    N = 1 << sf
    L = hdr.length
    h = [L >> 4, L & 0xF, (hdr.cr << 1) | int(hdr.has_crc)]
    chk = header_checksum(*h)
    h += [chk >> 4, chk & 0xF]
    blk = (h + list(hdr.extra_nibbles) + [0] * sf)[: sf - 2]
    vals = interleave([hamming_encode(x, 4) for x in blk], sf - 2, 8)
    return [(inv_gray(v) * 4 + 1) % N for v in vals]


def decode_header(bins: list[int], sf: int) -> LoRaHeader:
    N = 1 << sf
    sf_app = sf - 2
    vals = [gray(int(round(((b - 1) % N) / 4.0)) % (N // 4)) for b in bins[:8]]
    cws = deinterleave(vals, sf_app, 8)
    nibs, errs = [], 0
    for cw in cws:
        nb, e = hamming_decode(cw, 4)
        nibs.append(nb)
        errs += e
    length = (nibs[0] << 4) | nibs[1]
    cr = nibs[2] >> 1
    has_crc = bool(nibs[2] & 1)
    chk = ((nibs[3] & 1) << 4) | nibs[4]
    ok = chk == header_checksum(nibs[0], nibs[1], nibs[2]) and 1 <= cr <= 4 and length > 0
    return LoRaHeader(length, cr, has_crc, ok, nibs[5:], errs)


@dataclass
class LoRaPayload:
    data: bytes
    crc_ok: bool | None
    crc_rx: int | None
    crc_calc: int | None
    corrected_bits: int
    chase_fixed: int = 0


def decode_payload(bins: list[int], hdr: LoRaHeader, sf: int, ldro: bool) -> LoRaPayload:
    N = 1 << sf
    cr = hdr.cr
    sf_app = sf - 2 * int(ldro)
    cw_len = 4 + cr
    nibs = list(hdr.extra_nibbles)
    errs = 0
    data_bins = bins[8:]
    for k in range(0, len(data_bins) - cw_len + 1, cw_len):
        blk = data_bins[k:k + cw_len]
        if ldro:
            vals = [gray(int(round(((b - 1) % N) / 4.0)) % (N // 4)) for b in blk]
        else:
            vals = [gray((b - 1) % N) for b in blk]
        for cw in deinterleave(vals, sf_app, cw_len):
            nb, e = hamming_decode(cw, cr)
            nibs.append(nb)
            errs += e
    L = hdr.length
    raw = bytes((nibs[2 * i] | (nibs[2 * i + 1] << 4)) & 0xFF for i in range(min(len(nibs) // 2, L + 2)))
    data = bytes(raw[i] ^ int(WHITENING[i]) for i in range(min(L, len(raw))))
    crc_ok = crc_rx = crc_calc = None
    if hdr.has_crc and len(raw) >= L + 2:
        crc_rx = raw[L] | (raw[L + 1] << 8)
        crc_calc = lora_payload_crc(data)
        crc_ok = crc_rx == crc_calc
    return LoRaPayload(data, crc_ok, crc_rx, crc_calc, errs)


# ----------------------------------------------------------------------------
# Receptor em fluxo contínuo (amostras a fs = BW)
# ----------------------------------------------------------------------------


@dataclass
class LoRaFrame:
    start_sample: int          # índice absoluto (fs = BW) do início do preâmbulo estimado
    end_sample: int
    cfo_bins: float
    cfo_hz: float
    snr_db: float
    level_dbfs: float
    sync_word: int
    sync_bins: tuple
    preamble_syms: int
    header: LoRaHeader
    payload: LoRaPayload | None
    bins: list = field(default_factory=list)
    n_syms: int = 0
    ambiguity_resolved_by: str = ""
    clock_model: str = ""
    drift_bins: float = 0.0


def _peak_interp(mag: np.ndarray, k: int) -> float:
    N = len(mag)
    a, b, c = mag[(k - 1) % N], mag[k], mag[(k + 1) % N]
    den = a - 2 * b + c
    if den == 0:
        return float(k)
    return k + 0.5 * (a - c) / den


def _peak_frac(X: np.ndarray, k: int) -> float:
    """Posição fracionária do pico a partir da FFT complexa (estimador de Candan, janela retangular)."""
    N = len(X)
    a, b, c = X[(k - 1) % N], X[k], X[(k + 1) % N]
    den = 2 * b - a - c
    if den == 0:
        return float(k)
    d = np.real((a - c) / den)
    d *= np.tan(np.pi / N) / (np.pi / N)
    return k + float(np.clip(d, -0.5, 0.5))


_SLOPES = np.linspace(-0.9, 0.9, 1801)


def _fit_drift(pf: np.ndarray) -> tuple[float, float]:
    """Regressão circular: acha off + drift*i que melhor explica as partes fracionárias dos picos.
    Robusta a símbolos errados (cada um só contribui um vetor unitário)."""
    i = np.arange(len(pf))
    z = np.exp(2j * np.pi * (pf[None, :] - _SLOPES[:, None] * i[None, :]))
    s = z.sum(axis=1)
    k = int(np.argmax(np.abs(s)))
    off = float(np.angle(s[k]) / (2 * np.pi))
    return off, float(_SLOPES[k])


_SLOPES_H = np.linspace(-0.9, 0.9, 361)


def _fit_drift_period(v: np.ndarray, period: float):
    """Candidatos (deslocamento, deriva) para valores que são múltiplos de 'period' + ruído,
    em ordem de qualidade. O primeiro é o melhor ajuste; os seguintes cobrem ambiguidades."""
    i = np.arange(len(v))
    z = np.exp(2j * np.pi * (v[None, :] - _SLOPES_H[:, None] * i[None, :]) / period)
    sc = z.sum(axis=1)
    order = np.argsort(-np.abs(sc))
    out, seen = [], []
    for k in order[:40]:
        b_ = float(_SLOPES_H[k])
        if any(abs(b_ - x) < 0.04 for x in seen):
            continue
        seen.append(b_)
        out.append((float(np.angle(sc[k]) / (2 * np.pi) * period), b_))
        if len(out) >= 4:
            break
    return out


def _wrap_signed(x: float, N: int) -> float:
    return ((x + N / 2) % N) - N / 2


class LoRaReceiver:
    """Recebe blocos de IQ a fs = BW e devolve LoRaFrame decodificados."""

    def __init__(self, sf: int, bw: float, ldro: bool = False, min_preamble: int = 6,
                 detect_ratio: float = 10.0, max_payload: int = 255, os: int = 1,
                 carrier_hz: float | None = None):
        """os = sobreamostragem da entrada (fs = os*BW). carrier_hz permite compensar a deriva
        do relógio de amostragem a partir do erro de frequência (mesmo cristal)."""
        self.sf, self.bw, self.ldro = sf, bw, ldro
        self.os = int(os)
        self.carrier_hz = carrier_hz
        self.sfo_pref = None        # hipótese de relógio que funcionou por último
        self.sfo_known = None       # erro de relógio (fração) que decodificou o último pacote
        self.debug = None
        self.N = 1 << sf
        self.up, self.down = chirps(sf)
        self.min_preamble = min_preamble
        self.detect_ratio = detect_ratio
        self.max_frame = (20 + n_symbols(max_payload, sf, 1, True, ldro) + 4) * self.N
        self.buf = np.zeros(0, np.complex64)
        self.buf4_start = 0         # índice absoluto (a os*BW) de buf[0]
        self.grid = 0               # próxima janela a varrer (absoluto)
        self.run_start = self.run_len = 0
        self.run_bin = -1
        self.pending = None
        self.stats = {"preambulos": 0, "sync_falhou": 0, "cabecalho_invalido": 0, "quadros": 0}

    # -------------------------------------------------------------- buffer
    # Posições são medidas em chips (amostras a 1x); o buffer guarda amostras a os*BW.
    @property
    def buf_start(self) -> int:
        return -(-self.buf4_start // self.os)

    @property
    def buf_end(self) -> int:
        return (self.buf4_start + len(self.buf)) // self.os

    def _get(self, pos: float, n: int) -> np.ndarray:
        i = int(round(pos * self.os)) - self.buf4_start
        if i < 0:
            return np.zeros(0, np.complex64)
        return self.buf[i:i + n * self.os:self.os]

    def _trim(self):
        keep_from = self.grid - 2 * self.N
        if self.run_len:
            keep_from = min(keep_from, self.run_start - self.N)
        if self.pending is not None:
            keep_from = min(keep_from, self.pending["run_start"] - self.N)
        drop = keep_from * self.os - self.buf4_start
        if drop > 0:
            self.buf = self.buf[drop:]
            self.buf4_start += drop

    def feed(self, x: np.ndarray) -> list[LoRaFrame]:
        self.buf = np.concatenate([self.buf, x.astype(np.complex64)])
        return self._process(final=False)

    def flush(self) -> list[LoRaFrame]:
        return self._process(final=True)

    # -------------------------------------------------------------- varredura
    def _process(self, final: bool) -> list[LoRaFrame]:
        out = []
        N = self.N
        while True:
            if self.pending is not None:
                res = self._try_decode(self.pending, final)
                if res == "wait":
                    break
                frame, nxt = res
                if frame is not None:
                    out.append(frame)
                self.pending = None
                self.run_len = 0
                self.grid = max(self.grid, nxt)
                continue
            if self.grid + N > self.buf_end:
                if final and self.run_len >= self.min_preamble:
                    self.pending = dict(run_start=self.run_start, run_len=self.run_len, run_bin=self.run_bin)
                    self.run_len = 0
                    continue
                break
            if self.grid < self.buf_start:
                self.grid = self.buf_start
            w = self._get(self.grid, N)
            P = np.abs(np.fft.fft(w * self.down)) ** 2
            k = int(np.argmax(P))
            ratio = P[k] / (np.mean(P) + 1e-20)
            strong = ratio > self.detect_ratio
            near = self.run_len > 0 and min((k - self.run_bin) % N, (self.run_bin - k) % N) <= 2
            if strong and near:
                self.run_len += 1
                self.run_bin = k                 # acompanha a deriva de frequência do transmissor
                if self.run_len > 128:          # preâmbulo absurdo/interferência: janela deslizante
                    self.run_start += N
                    self.run_len -= 1
            else:
                if self.run_len >= self.min_preamble:
                    self.pending = dict(run_start=self.run_start, run_len=self.run_len, run_bin=self.run_bin)
                    self.stats["preambulos"] += 1
                if strong:
                    self.run_start, self.run_bin, self.run_len = self.grid, k, 1
                else:
                    self.run_len = 0
                if self.pending is not None:
                    self.run_len = 0
            self.grid += N
        self._trim()
        return out

    # -------------------------------------------------------------- sincronismo + decodificação
    def _try_decode(self, p, final: bool):
        N = self.N
        rs, rl = p["run_start"], p["run_len"]
        fail_next = rs + rl * N
        if rs < self.buf_start:
            return None, fail_next
        need = rs + (rl + 12) * N
        if need > self.buf_end:
            if final:
                return None, fail_next
            return "wait"

        # --- frequência do preâmbulo (parte inteira + fração pela fase entre janelas)
        Xs = [np.fft.fft(self._get(rs + i * N, N) * self.down) for i in range(rl)]
        # deriva de frequência medida no próprio preâmbulo (bins por símbolo)
        pre = [_peak_frac(X, int(np.argmax(np.abs(X)))) for X in Xs]
        dif = [_wrap_signed(pre[i] - pre[i - 1], N) for i in range(1, len(pre))]
        dif = [d for d in dif if abs(d) < 1.0]
        self._b_pre = float(np.median(dif)) if dif else 0.0
        # usa os últimos chirps do preâmbulo (mais perto do SFD, menos afetados pela deriva)
        Xs = Xs[-min(len(Xs), 6):]
        ks = [int(np.argmax(np.abs(X))) for X in Xs]
        f_a = float(np.mean([_peak_frac(X, k) for X, k in zip(Xs, ks)]))

        # --- procura janela totalmente dentro dos downchirps (SFD)
        best = None
        for j in range(max(rl - 2, 0), rl + 12):
            pos = rs + j * N
            w = self._get(pos, N)
            Pd = np.abs(np.fft.fft(w * self.up)) ** 2
            Pu = np.abs(np.fft.fft(w * self.down)) ** 2
            kd = int(np.argmax(Pd))
            if Pd[kd] > 2 * Pu.max() and (best is None or Pd[kd] > best[1]):
                best = (pos, Pd[kd], kd, Pd)
        if best is None or best[1] / (np.mean(best[3]) + 1e-20) < self.detect_ratio:
            self.stats["sync_falhou"] += 1
            return None, fail_next
        pos_d, _, kd, Pd = best
        f_b = _peak_frac(np.fft.fft(self._get(pos_d, N) * self.up), kd)

        s = (f_a + f_b) % N
        cands = [_wrap_signed(s / 2, N), _wrap_signed(s / 2 + N / 2, N)]
        results = []
        for c in cands:
            r = self._decode_with(c, f_b, pos_d, rs, rl, final)
            if r == "wait":
                return "wait"
            if r is not None:
                results.append(r)
        if not results:
            self.stats["cabecalho_invalido"] += 1
            return None, fail_next
        # preferir cabeçalho válido + CRC ok, depois menor |CFO|
        results.sort(key=lambda fr: (
            not fr.header.checksum_ok,
            not (fr.payload is not None and fr.payload.crc_ok),
            abs(fr.cfo_bins)))
        fr = results[0]
        fr.ambiguity_resolved_by = "checksum/CRC" if len(results) > 1 and results[0].header.checksum_ok != results[1].header.checksum_ok else "menor CFO"
        if not fr.header.checksum_ok:
            self.stats["cabecalho_invalido"] += 1
            return None, fail_next
        self.stats["quadros"] += 1
        return fr, fr.end_sample

    def _decide(self, hpfs, hdr: LoRaHeader, measure, nsym: int, chase: bool = False):
        """Decide os símbolos do payload acompanhando a deriva.
        1. deslocamento exato nos 8 símbolos do cabeçalho (re-codificado a partir do decodificado);
        2. para cada símbolo, prevê o deslocamento com uma reta local dos últimos decididos e
           aplica essa previsão na própria correção de frequência (measure(i, extra)), de modo que o
           resíduo fique perto de zero — evita o salto de fase no ponto de dobra do chirp;
        3. se o CRC falhar e chase=True, tenta os bins vizinhos dos símbolos mais ambíguos."""
        N, n = self.N, nsym
        ht = header_symbols(hdr, self.sf)
        absoff = np.array([_wrap_signed(hpfs[i] - ht[i], N) for i in range(8)])
        reduced = np.array([i < 8 or self.ldro for i in range(n)])
        hist_i = list(range(8))
        hist_r = list(absoff)
        bins = list(ht)
        o = np.zeros(n)
        o[:8] = absoff
        x = np.zeros(n)                            # resíduo (pf medido - previsão), para o Chase
        W = 12
        for i in range(8, n):
            hi = np.array(hist_i[-W:], dtype=float)
            hr = np.array(hist_r[-W:])
            if len(hi) >= 3:
                b_, a_ = np.polyfit(hi, hr, 1)
                pred = a_ + b_ * i
            else:
                pred = hr[-1]
            pf = measure(i, pred)                  # pico já com a previsão removida (perto de inteiro)
            if reduced[i]:
                kk = (4 * int(round((pf - 1) / 4)) + 1) % N
            else:
                kk = int(round(pf)) % N
            r = _wrap_signed(pf - kk, N)
            o[i] = pred
            x[i] = r
            bins.append(kk)
            if abs(r) < 0.45:
                hist_i.append(i)
                hist_r.append(pred + r)
        self._last_tslope = 0.0
        drift = float((o[-1] - o[0]) / (n - 1)) if n > 1 else 0.0
        pl = decode_payload(bins, hdr, self.sf, self.ldro)
        if pl.crc_ok or not hdr.has_crc or not chase:
            return pl, drift, bins
        # Chase: até 3 símbolos trocados pelo bin vizinho, entre os mais ambíguos
        frac = x
        cand = [i for i in np.argsort(-np.abs(frac)) if not reduced[i]][:12]
        tried = 0
        for r in (1, 2, 3):
            pool = cand if r < 3 else cand[:6]
            for combo in itertools.combinations(pool, r):
                for signs in itertools.product((1, -1), repeat=r):
                    tried += 1
                    b2 = list(bins)
                    for k, sg in zip(combo, signs):
                        b2[k] = (b2[k] + sg) % N
                    p2 = decode_payload(b2, hdr, self.sf, self.ldro)
                    if p2.crc_ok:
                        p2.chase_fixed = r
                        return p2, drift, b2
        return pl, drift, bins

    def _fold_peak(self, start: float, cfo_bins: float):
        """Demodula um símbolo na taxa sobreamostrada. O chirp "dá a volta" dentro do símbolo, e os
        dois pedaços caem nos bins k e k-N; somando a energia dos dois, um erro fracionário de tempo
        vira só um deslocamento suave do pico (sem a interferência que ocorre a 1 amostra/chip).
        Retorna (posição fracionária do pico em 0..N, energia)."""
        N, os_ = self.N, self.os
        if os_ < 2:
            X = np.fft.fft(self._corrected(start, N, cfo_bins) * self.down)
            k = int(np.argmax(np.abs(X)))
            return _peak_frac(X, k), float(np.abs(X[k]) ** 2)
        i0 = int(np.floor(start * os_))
        i = i0 - self.buf4_start
        M = os_ * N
        seg = self.buf[i:i + M] if i >= 0 else np.zeros(0, np.complex64)
        if len(seg) < M:
            seg = np.concatenate([seg, np.zeros(M - len(seg), np.complex64)])
        tt = (i0 + np.arange(M)) / os_                     # tempo absoluto em chips
        ph = (cfo_bins / N) * tt
        ph -= np.floor(ph)
        tau = tt - start                                   # tempo relativo ao início exato do símbolo
        ref = np.exp(-1j * (np.pi * tau * tau / N - np.pi * tau) - 2j * np.pi * ph)
        X = np.fft.fft(seg * ref.astype(np.complex64))
        lo = (np.arange(N) - N) % M
        P = np.abs(X[:N]) ** 2 + np.abs(X[lo]) ** 2
        k = int(np.argmax(P))
        idx = k if np.abs(X[k]) >= np.abs(X[lo[k]]) else int(lo[k])
        A, B, C = X[(idx - 1) % M], X[idx], X[(idx + 1) % M]
        den = 2 * B - A - C
        d = 0.0 if den == 0 else float(np.real((A - C) / den)) * np.tan(np.pi / M) / (np.pi / M)
        return k + float(np.clip(d, -0.5, 0.5)), float(P[k])

    def _corrected(self, start: float, n: int, cfo_bins: float) -> np.ndarray:
        """Janela de n chips com o desvio de frequência removido. Com sobreamostragem, a correção é
        feita na taxa alta e depois um filtro ideal (FFT) limita a ±BW/2 antes de decimar, de modo
        que desvios grandes (dezenas de kHz) não cortam o sinal."""
        os_ = self.os
        i0 = int(round(start * os_))
        i = i0 - self.buf4_start
        if i < 0:
            return np.zeros(0, np.complex64)
        x = self.buf[i:i + n * os_]
        if len(x) < n * os_:
            x = x[: (len(x) // os_) * os_]
            if len(x) == 0:
                return np.zeros(0, np.complex64)
        m = len(x) // os_
        idx = i0 / os_ + np.arange(len(x), dtype=np.float64) / os_
        ph = (cfo_bins / self.N) * idx
        ph -= np.floor(ph)
        y = x * np.exp(-2j * np.pi * ph).astype(np.complex64)
        X = np.fft.fft(y)
        M = len(X)
        # atraso fracionário exato (a posição foi arredondada para a amostra mais próxima):
        # meia amostra de erro já causa oscilação do pico dependente do valor do símbolo
        delta = start * os_ - i0
        if abs(delta) > 1e-6:
            kf = np.fft.fftfreq(M) * M
            X = X * np.exp(2j * np.pi * kf * delta / M).astype(np.complex64)
        if os_ == 1:
            return np.fft.ifft(X).astype(np.complex64)
        h = m // 2
        Z = np.concatenate([X[:m - h], X[M - h:]])
        return (np.fft.ifft(Z) / os_).astype(np.complex64)

    def _bin(self, x: np.ndarray, ref: np.ndarray):
        P = np.abs(np.fft.fft(x * ref)) ** 2
        k = int(np.argmax(P))
        return k, P

    def _decode_with(self, c: float, f_b: float, pos_d: int, rs: int, rl: int, final: bool):
        """Tenta as hipóteses de deriva de relógio; fica com a primeira que passa no CRC."""
        coh = (c * self.bw / self.N) / self.carrier_hz if self.carrier_hz else 0.0
        hyps = [coh, 0.0] if coh else [0.0]
        if self.sfo_pref is not None and self.sfo_pref < len(hyps):
            hyps.insert(0, hyps.pop(self.sfo_pref))
        if self.sfo_known is not None:            # erro de relógio que já funcionou antes
            hyps = [self.sfo_known] + [h for h in hyps if abs(h - self.sfo_known) > 2e-6]
        first = None
        first_eps = None
        tried = 0
        self._chase_ok = False                    # Chase (caro) só uma vez, depois das hipóteses
        while tried < len(hyps):
            eps = hyps[tried]
            tried += 1
            r = self._decode_eps(c, f_b, pos_d, rs, rl, final, eps)
            if r == "wait" or r is None:
                return r
            if first is None:
                first, first_eps = r, eps
            if not r.header.checksum_ok:
                return r
            if r.payload is not None and r.payload.crc_ok:
                self.sfo_known = eps
                if eps == coh and coh:
                    self.sfo_pref, r.clock_model = 0, "relógio acompanha o erro de frequência"
                elif eps == 0.0:
                    self.sfo_pref, r.clock_model = (1 if coh else 0), "relógio independente"
                else:
                    r.clock_model = f"deriva medida {eps * 1e6:+.1f} ppm"
                return r
            # novas hipóteses: escorregamento de tempo medido no pacote (mais confiável) e a deriva
            # residual dos bins
            if len(hyps) < 9:
                ts = getattr(self, "_last_tslope", 0.0) / self.N
                d = r.drift_bins / self.N
                for e in (eps + ts, eps - ts, eps - d, eps + d):
                    if abs(e - eps) > 2e-6 and all(abs(e - h) > 2e-6 for h in hyps):
                        hyps.append(e)
        if first is not None and first.header.checksum_ok and first.header.has_crc:
            self._chase_ok = True
            r = self._decode_eps(c, f_b, pos_d, rs, rl, final, first_eps)
            self._chase_ok = False
            if r not in (None, "wait") and r.payload is not None and r.payload.crc_ok:
                r.clock_model = "recuperado pelo CRC"
                return r
        return first

    def _decode_eps(self, c, f_b, pos_d, rs, rl, final, eps):
        N = self.N
        tau = (f_b - c) % N                       # deslocamento de tempo em chips (fracionário)
        k = 1.0 / (1.0 + eps)                     # amostras por chip nominal
        # tau é relativo à grade de varredura (que começa em rs): bordas dos chirps em rs + tau + k*N
        p0 = pos_d - ((pos_d - rs - tau) % N)     # maior posição alinhada <= pos_d
        # 1º downchirp: janela "down" seguida de outra "down" e precedida de uma que não é
        def is_down(pos):
            if pos < self.buf_start or pos + N > self.buf_end:
                return None
            x = self._corrected(pos, N, c)
            _, Pd = self._bin(x, self.up)
            _, Pu = self._bin(x, self.down)
            return Pd.max() > 2 * Pu.max()

        cands = [p0 + m * N for m in range(-4, 3)]
        flags = [is_down(q) for q in cands]
        D = None
        for m in range(1, len(cands) - 1):
            if flags[m] and flags[m + 1] and flags[m - 1] is False:
                D = cands[m]
                break
        if D is None:
            for m, f in enumerate(flags):
                if f:
                    D = cands[m]
                    break
        if D is None:
            return None
        # refinamento fino: no domínio alinhado, upchirp cai em -erro e downchirp em +erro
        ups = [self._corrected(D - i * N, N, c) for i in (3, 4, 5) if D - i * N >= max(rs, self.buf_start)]
        downs = [self._corrected(D + i * N, N, c) for i in (0, 1)]
        if ups:
            def fr_of(v, ref):
                X = np.fft.fft(v * ref)
                return _wrap_signed(_peak_frac(X, int(np.argmax(np.abs(X)))), N)
            fu = np.mean([fr_of(u, self.down) for u in ups])
            fd = np.mean([fr_of(d, self.up) for d in downs])
            # ups e downs são medidos em instantes diferentes; desconta a deriva de frequência
            # medida no preâmbulo para ela não virar erro de tempo
            t_u = -np.mean([i for i in (3, 4, 5) if D - i * N >= max(rs, self.buf_start)])
            dt_sym = 0.5 - t_u
            bp = getattr(self, "_b_pre", 0.0)
            if abs(fu) < 3 and abs(fd) < 3:
                D += (fd - fu - bp * dt_sym) / 2
                c += (fd + fu + bp * dt_sym) / 2

        D0 = D
        best = None
        for dD in (0, -N, N):                     # o 1º downchirp pode ter sido escolhido um símbolo fora
            D = D0 + dD

            def sym_pos(i, D=D):
                return D + (2.25 * N + i * N) * k

            if sym_pos(8) > self.buf_end:
                return None if final else "wait"

            def peak(i, extra=0.0, sym_pos=sym_pos):
                return self._fold_peak(sym_pos(i), c + extra)[0]

            pfs = [peak(i) for i in range(8)]
            hp = np.array(pfs)
            # candidatos de (deslocamento, deriva) para o cabeçalho. Os valores do cabeçalho são 4m+1,
            # então só se enxerga o deslocamento módulo 4: a deriva medida no preâmbulo é extrapolada
            # para prever de que lado da ambiguidade ele está, e o CRC do payload confirma.
            bp = getattr(self, "_b_pre", 0.0)
            lines = []
            for b_ in (bp, bp + eps * N, bp - eps * N):
                lines.append((b_ * 4.5, b_))
            for a4, b4 in _fit_drift_period(hp - 1, 4.0):
                for sh in (0.0, 4.0, -4.0):
                    lines.append((a4 + sh, b4))
            for a_, b_ in lines:
                # ajuste fino do deslocamento ao redor da previsão (resíduo módulo 4)
                pred = a_ + b_ * np.arange(8)
                r4 = np.angle(np.sum(np.exp(2j * np.pi * (hp - 1 - pred) / 4))) / (2 * np.pi) * 4
                o_h = pred + r4
                hb = [(4 * int(round((pf - 1 - o_h[i]) / 4)) + 1) % N for i, pf in enumerate(hp)]
                h_ = decode_header(hb, self.sf)
                if not h_.checksum_ok:
                    if best is None:
                        best = (hb, h_, D, sym_pos, peak, pfs, None)
                    continue
                nsym_ = n_symbols(h_.length, self.sf, h_.cr, h_.has_crc, self.ldro)
                if sym_pos(nsym_) > self.buf_end:
                    if final:
                        continue
                    return "wait"
                pl_, dr_, bins_ = self._decide(hp, h_, peak, nsym_)
                cand = (hb, h_, D, sym_pos, peak, pfs, (pl_, dr_, bins_))
                if best is None or not best[1].checksum_ok:
                    best = cand
                if pl_.crc_ok or not h_.has_crc:
                    best = cand
                    break
            if best is not None and best[6] is not None and (best[6][0].crc_ok or not best[1].has_crc):
                break
        def nsym_of(h):
            return n_symbols(h.length, self.sf, h.cr, h.has_crc, self.ldro)

        bins, hdr, D, sym_pos, peak, pfs, done = best
        self._last_sym0 = sym_pos(0)
        if done is not None and not done[0].crc_ok and hdr.has_crc and getattr(self, "_chase_ok", True):
            done = self._decide(np.array(pfs), hdr, peak, nsym_of(hdr), chase=True)
        nsym = n_symbols(hdr.length, self.sf, hdr.cr if 1 <= hdr.cr <= 4 else 4, hdr.has_crc, self.ldro)
        end = int(math.ceil(sym_pos(nsym)))
        drift = 0.0
        if not hdr.checksum_ok:
            fr_payload = None
        else:
            if end > self.buf_end:
                return None if final else "wait"
            fr_payload, drift, bins = done

        # sync word, SNR e nível usando o preâmbulo alinhado
        sync_bins = []
        for q in (D - 2 * N, D - N):
            if q >= self.buf_start:
                kk, _ = self._bin(self._corrected(q, N, c), self.down)
                sync_bins.append(kk)
        sw = 0
        if len(sync_bins) == 2:
            sw = ((int(round(sync_bins[0] / 8)) & 0xF) << 4) | (int(round(sync_bins[1] / 8)) & 0xF)
        snrs, lvl = [], []
        for i in range(3, min(rl, 10) + 2):
            q = D - i * N
            if q < self.buf_start or q < rs:
                break
            bp = getattr(self, "_b_pre", 0.0)
            x = self._corrected(q, N, c + bp * (0.5 - i - 1.0))
            P = np.abs(np.fft.fft(x * self.down)) ** 2
            sig = P[0] + P[1] + P[-1]
            noise_bin = np.median(P) / np.log(2)
            snrs.append(max(sig - 3 * noise_bin, 1e-12) / (noise_bin * N))
            lvl.append(np.mean(np.abs(x) ** 2))
        snr_db = 10 * np.log10(np.mean(snrs)) if snrs else float("nan")
        level = 10 * np.log10(np.mean(lvl) + 1e-20) if lvl else float("nan")
        n_pre = int(round((D - 2 * N - (rs - ((-tau) % N))) / N))
        start = int(D - (2 + max(n_pre, rl)) * N)
        return LoRaFrame(
            start_sample=start, end_sample=end, cfo_bins=c, cfo_hz=c * self.bw / N,
            snr_db=float(snr_db), level_dbfs=float(level), sync_word=sw, sync_bins=tuple(sync_bins),
            preamble_syms=max(n_pre, rl), header=hdr, payload=fr_payload, bins=bins, n_syms=nsym,
            drift_bins=drift)
