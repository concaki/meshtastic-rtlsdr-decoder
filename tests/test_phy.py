"""LoRa PHY: coding round trip and receiver robustness (CFO, clock drift, frequency drift, noise)."""
import numpy as np
import pytest
from scipy.signal import resample_poly

from lora_phy import (LoRaReceiver, decode_header, decode_payload, encode_symbols, header_checksum,
                      lora_payload_crc, modulate, n_symbols, time_on_air_ms)
from sources import FrontEnd

SF, BW, CR = 11, 250e3, 1
N = 1 << SF


def test_header_checksum_detects_single_bit_errors():
    # every single-bit change in the 12 header bits must change the 5-bit checksum
    for n0 in range(16):
        for n1 in (0, 7, 11):
            for n2 in (2, 3, 9):
                ref = header_checksum(n0, n1, n2)
                for bit in range(12):
                    m = [n0, n1, n2]
                    m[bit // 4] ^= 1 << (bit % 4)
                    assert header_checksum(*m) != ref


def test_crc_matches_semtech_definition():
    data = bytes(range(10))
    crc = lora_payload_crc(data)
    assert 0 <= crc <= 0xFFFF
    assert lora_payload_crc(data[:-1] + bytes([data[-1] ^ 1])) != crc


@pytest.mark.parametrize("length", [1, 16, 27, 100, 237])
@pytest.mark.parametrize("cr", [1, 2, 3, 4])
def test_symbol_round_trip(length, cr):
    rng = np.random.default_rng(length * 10 + cr)
    pl = bytes(rng.integers(0, 256, length, dtype=np.uint8))
    bins = encode_symbols(pl, SF, cr)
    assert len(bins) == n_symbols(length, SF, cr, True, False)
    hdr = decode_header(bins, SF)
    assert hdr.checksum_ok and hdr.length == length and hdr.cr == cr and hdr.has_crc
    out = decode_payload(bins, hdr, SF, False)
    assert out.data == pl and out.crc_ok


def test_time_on_air_longfast():
    # 27 bytes LongFast ~ 0.44 s on air (16-symbol preamble)
    assert 400 < time_on_air_ms(27, SF, BW, CR, 16) < 480


def _make(length, cfo_hz=0.0, sfo_ppm=0.0, drift_hz_s=0.0, snr_db=None, seed=0):
    rng = np.random.default_rng(seed)
    pl = bytes(rng.integers(0, 256, length, dtype=np.uint8))
    x = modulate(pl, SF, CR)
    x = np.concatenate([np.zeros(4000, np.complex64), x, np.zeros(30000, np.complex64)])
    fs = 2e6
    u = resample_poly(x, 8, 1)[3:]
    n = np.arange(len(u))
    if sfo_ppm:
        t = n * (1 + sfo_ppm * 1e-6)
        u = np.interp(t, n, u.real) + 1j * np.interp(t, n, u.imag)
    t = n / fs
    s = u * np.exp(1j * 2 * np.pi * ((500e3 + cfo_hz) * t + 0.5 * drift_hz_s * t ** 2))
    if snr_db is not None:
        sig = np.sqrt(np.mean(np.abs(u) ** 2))
        sigma = sig * 10 ** (-snr_db / 20) * np.sqrt(8) / np.sqrt(2)
        s = s + sigma * (rng.standard_normal(len(s)) + 1j * rng.standard_normal(len(s)))
    return pl, s.astype(np.complex64)


def _decode(s):
    fe = FrontEnd(2e6, 2, 500e3, BW)
    rx = LoRaReceiver(SF, BW, os=4, carrier_hz=919.875e6)
    frames = []
    for i in range(0, len(s), 131072):
        frames += rx.feed(fe.process(s[i:i + 131072]))
    frames += rx.flush()
    return frames


def _ok(frames, pl):
    return any(f.payload is not None and f.payload.crc_ok and f.payload.data == pl for f in frames)


@pytest.mark.parametrize("cfo", [0.0, 25e3, -51e3])
def test_large_frequency_offset(cfo):
    pl, s = _make(40, cfo_hz=cfo, seed=1)
    assert _ok(_decode(s), pl)


@pytest.mark.parametrize("sfo", [55.0, 0.0, -55.0, 25.0])
def test_sample_clock_offset_independent_of_cfo(sfo):
    # cheap R828D clones: tuner and ADC use different crystals
    pl, s = _make(120, cfo_hz=51e3, sfo_ppm=sfo, seed=2)
    assert _ok(_decode(s), pl)


@pytest.mark.parametrize("drift", [-4200.0, 4200.0])
def test_frequency_drift_during_packet(drift):
    pl, s = _make(200, cfo_hz=51e3, drift_hz_s=drift, snr_db=5, seed=3)
    assert _ok(_decode(s), pl)


def test_low_snr():
    pl, s = _make(60, cfo_hz=10e3, snr_db=-8, seed=4)
    assert _ok(_decode(s), pl)


def test_no_false_alarms_on_noise():
    rng = np.random.default_rng(0)
    rx = LoRaReceiver(SF, BW, os=4, carrier_hz=919.875e6)
    frames = []
    for i in range(40):
        x = (rng.standard_normal(250000) + 1j * rng.standard_normal(250000)).astype(np.complex64)
        if 10 <= i < 20:  # continuous carrier interference
            x += 3 * np.exp(2j * np.pi * 0.013 * np.arange(250000)).astype(np.complex64)
        frames += rx.feed(x)
    assert frames == []
    assert len(rx.buf) < 100000  # buffer does not grow without bound
