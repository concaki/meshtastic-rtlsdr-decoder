"""Meshtastic layer: header, channel hash, AES-CTR, AEAD channels, PKI direct messages."""
import struct

from cryptography.hazmat.primitives.asymmetric.x25519 import X25519PrivateKey
from cryptography.hazmat.primitives.ciphers.aead import AESCCM
from cryptography.hazmat.primitives.serialization import Encoding, NoEncryption, PrivateFormat, PublicFormat
from meshtastic.protobuf import mesh_pb2, portnums_pb2

from mesh_decode import (DEFAULT_PSK, ChannelKey, KeyRing, NodeDB, _nonce, aes_ctr, channel_hash,
                         decode_content, decrypt, expand_psk, parse_header, parse_psk_arg, pki_shared_key)

A, B = 0xA1B2C3D4, 0x0BADF00D


def _data(text: str) -> bytes:
    return mesh_pb2.Data(portnum=portnums_pb2.PortNum.TEXT_MESSAGE_APP, payload=text.encode()).SerializeToString()


def _hdr(to, frm, pid, chash, flags=0x63):
    return struct.pack("<III", to, frm, pid) + bytes([flags, chash, 0, frm & 0xFF])


def test_default_psk_expansion_and_longfast_hash():
    assert expand_psk(b"\x01") == DEFAULT_PSK
    assert expand_psk(b"\x02")[-1] == (DEFAULT_PSK[-1] + 1) & 0xFF
    assert expand_psk(b"\x00") == b""
    assert channel_hash("LongFast", DEFAULT_PSK) == 0x08
    assert parse_psk_arg("AQ==") == b"\x01"


def test_anz_longfast_frequency_slot():
    h = 5381
    for c in b"LongFast":
        h = (h * 33 + c) & 0xFFFFFFFF
    assert 915.0 + 0.125 + (h % 52) * 0.25 == 919.875


def test_header_flags():
    h = parse_header(_hdr(0xFFFFFFFF, A, 0x1234, 8, flags=0x6B) + b"x")
    assert h.hop_limit == 3 and h.hop_start == 3 and h.want_ack and not h.via_mqtt


def test_longfast_text_decrypt():
    raw = _hdr(0xFFFFFFFF, A, 0x1001, 0x08) + aes_ctr(DEFAULT_PSK, A, 0x1001, _data("Olá"))
    ring = KeyRing()
    ring.add(ChannelKey("LongFast", DEFAULT_PSK))
    h = parse_header(raw)
    d = decrypt(h, raw[16:], ring, NodeDB())
    assert d.data is not None and decode_content(d.data)["texto"] == "Olá"


def test_private_channel_and_unknown_key():
    key = bytes(range(32))
    raw = _hdr(0xFFFFFFFF, A, 7, channel_hash("Familia", key)) + aes_ctr(key, A, 7, _data("oi"))
    ring = KeyRing()
    ring.add(ChannelKey("LongFast", DEFAULT_PSK))
    h = parse_header(raw)
    assert decrypt(h, raw[16:], ring, NodeDB()).data is None
    ring.add(ChannelKey("Familia", key))
    assert decode_content(decrypt(h, raw[16:], ring, NodeDB()).data)["texto"] == "oi"


def test_aead_channel():
    key = bytes(range(16))
    h_ = _hdr(0xFFFFFFFF, A, 9, channel_hash("Seguro", key, True))
    ct = AESCCM(key, tag_length=12).encrypt(_nonce(A, 9)[:13], _data("ccm"), struct.pack("<II", A, 0xFFFFFFFF))
    ring = KeyRing()
    ring.add(ChannelKey("Seguro", key, aead=True))
    h = parse_header(h_ + ct)
    d = decrypt(h, ct, ring, NodeDB())
    assert d.authenticated and decode_content(d.data)["texto"] == "ccm"


def test_pki_direct_message():
    def kp():
        sk = X25519PrivateKey.generate()
        return (sk.private_bytes(Encoding.Raw, PrivateFormat.Raw, NoEncryption()),
                sk.public_key().public_bytes(Encoding.Raw, PublicFormat.Raw))
    a_priv, a_pub = kp()
    b_priv, b_pub = kp()
    extra = 0x5EED1234
    ct = AESCCM(pki_shared_key(b_priv, a_pub), tag_length=8).encrypt(_nonce(B, 0x22, extra)[:13], _data("dm"), None)
    payload = ct + struct.pack("<I", extra)
    h = parse_header(_hdr(A, B, 0x22, 0) + payload)
    db = NodeDB()
    db.my_num = A
    db.get(B).public_key = b_pub
    ring = KeyRing()
    ring.private_key = a_priv
    d = decrypt(h, payload, ring, db)
    assert d.pki and decode_content(d.data)["texto"] == "dm"
