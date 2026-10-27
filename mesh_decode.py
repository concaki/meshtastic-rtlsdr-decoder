# SPDX-License-Identifier: GPL-3.0-or-later
"""
Camada Meshtastic: cabeçalho on-air, decifragem e decodificação dos protobufs.

Conferido com o firmware (src/mesh/RadioInterface.h, CryptoEngine.cpp, Channels.cpp, Router.cpp):
  - cabeçalho de 16 bytes: to, from, id (LE32), flags, channel_hash, next_hop, relay_node
  - canal: AES-CTR, nonce = id (u64 LE) + from (u32 LE) + 4 zeros
  - canal AEAD (use_aead): AES-CCM tag 12, nonce 13 bytes, AAD = from+to, hash ^= 0xAE
  - PKI (DM): X25519 + SHA256 -> AES-256-CCM tag 8; payload = cifra || tag(8) || extraNonce(4)
"""
from __future__ import annotations

import base64
import hashlib
import struct
import time
from dataclasses import dataclass, field

from cryptography.hazmat.primitives.asymmetric.x25519 import X25519PrivateKey, X25519PublicKey
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.primitives.ciphers.aead import AESCCM
from google.protobuf.json_format import MessageToDict

try:
    from meshtastic.protobuf import mesh_pb2, portnums_pb2
except ImportError:  # versões antigas da lib
    from meshtastic import mesh_pb2, portnums_pb2  # type: ignore

try:
    from meshtastic import protocols as MT_PROTOCOLS
except Exception:  # pragma: no cover
    MT_PROTOCOLS = {}

BROADCAST = 0xFFFFFFFF
DEFAULT_PSK = bytes([0xD4, 0xF1, 0xBB, 0x3A, 0x20, 0x29, 0x07, 0x59,
                     0xF0, 0xBC, 0xFF, 0xAB, 0xCF, 0x4E, 0x69, 0x01])

PRESETS = {
    # nome: (bw Hz, sf, cr 1..4)
    "ShortTurbo": (500e3, 7, 1), "ShortFast": (250e3, 7, 1), "ShortSlow": (250e3, 8, 1),
    "MediumFast": (250e3, 9, 1), "MediumSlow": (250e3, 10, 1), "MediumTurbo": (500e3, 9, 1),
    "LongFast": (250e3, 11, 1), "LongTurbo": (500e3, 11, 4), "LongMod": (125e3, 11, 4),
    "LongSlow": (125e3, 12, 4), "LiteFast": (125e3, 9, 1), "LiteSlow": (125e3, 10, 1),
    "NarrowFast": (62.5e3, 7, 2), "NarrowSlow": (62.5e3, 8, 2),
}
PRESET_ENUM_TO_NAME = {
    "LONG_FAST": "LongFast", "LONG_SLOW": "LongSlow", "VERY_LONG_SLOW": "VLongSlow",
    "MEDIUM_SLOW": "MediumSlow", "MEDIUM_FAST": "MediumFast", "SHORT_SLOW": "ShortSlow",
    "SHORT_FAST": "ShortFast", "LONG_MODERATE": "LongMod", "SHORT_TURBO": "ShortTurbo",
    "LONG_TURBO": "LongTurbo", "MEDIUM_TURBO": "MediumTurbo", "LITE_FAST": "LiteFast",
    "LITE_SLOW": "LiteSlow", "NARROW_FAST": "NarrowFast", "NARROW_SLOW": "NarrowSlow",
}


def node_id(num: int) -> str:
    return "^all" if num == BROADCAST else f"!{num:08x}"


def expand_psk(psk: bytes) -> bytes:
    """Expande a forma curta (1 byte) da PSK como o firmware faz. b'' = sem cifra."""
    if len(psk) == 0:
        return b""
    if len(psk) == 1:
        idx = psk[0]
        if idx == 0:
            return b""
        k = bytearray(DEFAULT_PSK)
        k[-1] = (k[-1] + idx - 1) & 0xFF
        return bytes(k)
    if len(psk) < 16:
        return psk + bytes(16 - len(psk))
    if 16 < len(psk) < 32:
        return psk + bytes(32 - len(psk))
    return psk


def xor_hash(b: bytes) -> int:
    h = 0
    for x in b:
        h ^= x
    return h


def channel_hash(name: str, key: bytes, aead: bool = False) -> int:
    h = xor_hash(name.encode()) ^ xor_hash(key)
    return h ^ 0xAE if aead else h


@dataclass
class ChannelKey:
    name: str
    key: bytes              # já expandida
    aead: bool = False
    source: str = ""
    index: int | None = None

    @property
    def hash(self) -> int:
        return channel_hash(self.name, self.key, self.aead)

    def describe(self) -> str:
        if not self.key:
            kind = "sem cifra"
        else:
            kind = f"AES-{len(self.key) * 8}" + ("-CCM" if self.aead else "-CTR")
        idx = f"#{self.index} " if self.index is not None else ""
        return f"{idx}{self.name} ({kind}, hash 0x{self.hash:02x}, {self.source})"


def parse_psk_arg(text: str) -> bytes:
    """Aceita base64 (formato do app), 'default'/'AQ==', 'none', ou hex com prefixo 0x."""
    t = text.strip()
    if t.lower() in ("default", "padrao", "padrão"):
        return b"\x01"
    if t.lower() in ("none", "nenhuma", "sem"):
        return b"\x00"
    if t.lower().startswith("0x"):
        return bytes.fromhex(t[2:])
    if t.startswith("base64:"):
        t = t[7:]
    return base64.b64decode(t)


@dataclass
class NodeInfo:
    num: int
    long_name: str = ""
    short_name: str = ""
    hw_model: str = ""
    role: str = ""
    public_key: bytes = b""
    source: str = ""
    last_heard: float = 0.0
    position: tuple | None = None

    def label(self) -> str:
        if self.long_name:
            return f"{node_id(self.num)} {self.long_name} [{self.short_name}]"
        return node_id(self.num)


class NodeDB:
    def __init__(self):
        self.nodes: dict[int, NodeInfo] = {}
        self.my_num: int | None = None

    def get(self, num: int) -> NodeInfo:
        if num not in self.nodes:
            self.nodes[num] = NodeInfo(num)
        return self.nodes[num]

    def label(self, num: int) -> str:
        if num == BROADCAST:
            return "^all (broadcast)"
        n = self.nodes.get(num)
        lab = n.label() if n else node_id(num)
        if self.my_num is not None and num == self.my_num:
            lab += " (seu nó)"
        return lab

    def by_last_byte(self, b: int) -> list[NodeInfo]:
        return [n for n in self.nodes.values() if (n.num & 0xFF) == b]

    def update_user(self, num: int, user: dict, source: str):
        n = self.get(num)
        n.long_name = user.get("long_name", n.long_name) or n.long_name
        n.short_name = user.get("short_name", n.short_name) or n.short_name
        n.hw_model = user.get("hw_model", n.hw_model) or n.hw_model
        n.role = user.get("role", n.role) or n.role
        pk = user.get("public_key")
        if pk:
            n.public_key = base64.b64decode(pk) if isinstance(pk, str) else bytes(pk)
        n.source = n.source or source
        n.last_heard = time.time()


class KeyRing:
    def __init__(self):
        self.channels: list[ChannelKey] = []
        self.private_key: bytes = b""

    def add(self, ch: ChannelKey):
        for c in self.channels:
            if c.name == ch.name and c.key == ch.key and c.aead == ch.aead:
                return
        self.channels.append(ch)


# ----------------------------------------------------------------------------
# Cabeçalho e decifragem
# ----------------------------------------------------------------------------


@dataclass
class MeshHeader:
    to: int
    frm: int
    id: int
    flags: int
    channel: int
    next_hop: int
    relay_node: int

    @property
    def hop_limit(self) -> int:
        return self.flags & 0x07

    @property
    def want_ack(self) -> bool:
        return bool(self.flags & 0x08)

    @property
    def via_mqtt(self) -> bool:
        return bool(self.flags & 0x10)

    @property
    def hop_start(self) -> int:
        return (self.flags >> 5) & 0x07


def parse_header(frame: bytes) -> MeshHeader | None:
    if len(frame) < 16:
        return None
    to, frm, pid = struct.unpack_from("<III", frame, 0)
    return MeshHeader(to, frm, pid, frame[12], frame[13], frame[14], frame[15])


def _nonce(frm: int, pid: int, extra: int = 0) -> bytes:
    n = bytearray(16)
    n[0:8] = struct.pack("<Q", pid)
    n[8:12] = struct.pack("<I", frm)
    if extra:
        n[4:8] = struct.pack("<I", extra)
    return bytes(n)


def aes_ctr(key: bytes, frm: int, pid: int, data: bytes) -> bytes:
    c = Cipher(algorithms.AES(key), modes.CTR(_nonce(frm, pid))).decryptor()
    return c.update(data) + c.finalize()


def aes_ccm_channel(key: bytes, h: MeshHeader, data: bytes) -> bytes | None:
    if len(data) <= 12 or len(key) not in (16, 32):
        return None
    aad = struct.pack("<II", h.frm, h.to)
    try:
        return AESCCM(key, tag_length=12).decrypt(_nonce(h.frm, h.id)[:13], data, aad)
    except Exception:
        return None


def pki_shared_key(priv: bytes, pub: bytes) -> bytes:
    sk = X25519PrivateKey.from_private_bytes(priv)
    shared = sk.exchange(X25519PublicKey.from_public_bytes(pub))
    return hashlib.sha256(shared).digest()


def pki_decrypt(priv: bytes, peer_pub: bytes, h: MeshHeader, data: bytes) -> bytes | None:
    if len(data) <= 12 or len(priv) != 32 or len(peer_pub) != 32:
        return None
    ct_tag, extra = data[:-4], struct.unpack("<I", data[-4:])[0]
    try:
        key = pki_shared_key(priv, peer_pub)
        return AESCCM(key, tag_length=8).decrypt(_nonce(h.frm, h.id, extra)[:13], ct_tag, None)
    except Exception:
        return None


def _parse_data(raw: bytes):
    d = mesh_pb2.Data()
    try:
        d.ParseFromString(raw)
    except Exception:
        return None
    if d.portnum == 0:
        return None
    if d.portnum not in portnums_pb2.PortNum.values():
        return None
    return d


@dataclass
class Decryption:
    data: object | None = None     # mesh_pb2.Data
    method: str = ""               # descrição legível
    channel: ChannelKey | None = None
    pki: bool = False
    authenticated: bool = False
    note: str = ""


def decrypt(h: MeshHeader, enc: bytes, ring: KeyRing, db: NodeDB) -> Decryption:
    # 1) PKI (DM): hash 0 e destino unicast
    if h.channel == 0 and h.to != BROADCAST and len(enc) > 12:
        priv = ring.private_key
        if priv and db.my_num is not None:
            peer = None
            if h.to == db.my_num and h.frm in db.nodes:
                peer = db.nodes[h.frm].public_key
            elif h.frm == db.my_num and h.to in db.nodes:
                peer = db.nodes[h.to].public_key
            if peer:
                raw = pki_decrypt(priv, peer, h, enc)
                if raw is not None:
                    d = _parse_data(raw)
                    if d is not None:
                        return Decryption(d, "PKI X25519 + AES-256-CCM (mensagem direta)", None, True, True)
                    return Decryption(None, "PKI", note="CCM autenticou mas o protobuf não decodificou")
        note = "mensagem direta cifrada com PKI (X25519)"
        if not priv:
            note += "; sem chave privada do seu nó"
        elif h.to != db.my_num and h.frm != db.my_num:
            note += "; não é de/para o seu nó, impossível decifrar"
        else:
            note += "; chave pública do outro nó ainda desconhecida"
        pki_note = note
    else:
        pki_note = ""

    matches = [c for c in ring.channels if c.hash == h.channel]
    others = [c for c in ring.channels if c.hash != h.channel]
    for group, label in ((matches, ""), (others, " (hash não bate, tentativa forçada)")):
        for ch in group:
            if not ch.key:
                raw = enc
            elif ch.aead:
                raw = aes_ccm_channel(ch.key, h, enc)
                if raw is None:
                    continue
            else:
                raw = aes_ctr(ch.key, h.frm, h.id, enc)
            d = _parse_data(raw)
            if d is not None:
                kind = "sem cifra" if not ch.key else (f"AES-{len(ch.key) * 8}-" + ("CCM" if ch.aead else "CTR"))
                return Decryption(d, f"canal '{ch.name}' {kind}{label}", ch, False, ch.aead)
    if pki_note:
        return Decryption(None, "", note=pki_note)
    return Decryption(None, "", note=f"nenhuma chave conhecida para o hash de canal 0x{h.channel:02x}")


# ----------------------------------------------------------------------------
# Conteúdo por porta
# ----------------------------------------------------------------------------

TEXT_PORTS = {"TEXT_MESSAGE_APP", "RANGE_TEST_APP", "DETECTION_SENSOR_APP", "ALERT_APP", "REPLY_APP"}


def port_name(p: int) -> str:
    try:
        return portnums_pb2.PortNum.Name(p)
    except ValueError:
        return f"PORTA_{p}"


def _factory_for(portnum: int):
    proto = MT_PROTOCOLS.get(portnum) if hasattr(MT_PROTOCOLS, "get") else None
    if proto is not None and getattr(proto, "protobufFactory", None):
        return proto.protobufFactory
    return None


def decode_content(d) -> dict:
    """Retorna {'tipo': ..., 'texto'|'campos'|'hex': ...}."""
    pname = port_name(d.portnum)
    out = {"porta": pname, "porta_num": d.portnum}
    payload = bytes(d.payload)
    if pname in TEXT_PORTS:
        out["texto"] = payload.decode("utf-8", "replace")
        return out
    fac = _factory_for(d.portnum)
    if fac is not None:
        try:
            m = fac()
            m.ParseFromString(payload)
            out["campos"] = MessageToDict(m, preserving_proto_field_name=True)
            return out
        except Exception as e:  # noqa: BLE001
            out["erro"] = f"falha ao decodificar protobuf: {e}"
    out["hex"] = payload.hex()
    if pname == "TEXT_MESSAGE_COMPRESSED_APP":
        out["obs"] = "texto comprimido (Unishox2) — não descomprimido"
    return out


def data_meta(d) -> dict:
    m = {}
    if d.want_response:
        m["want_response"] = True
    if d.dest:
        m["dest"] = node_id(d.dest)
    if d.source:
        m["source"] = node_id(d.source)
    if d.request_id:
        m["request_id"] = f"0x{d.request_id:08x}"
    if d.reply_id:
        m["reply_id"] = f"0x{d.reply_id:08x}"
    if d.emoji:
        m["emoji"] = d.emoji
    if d.HasField("bitfield"):
        m["bitfield"] = d.bitfield
        m["ok_to_mqtt"] = bool(d.bitfield & 1)
    try:
        from google.protobuf import unknown_fields
        unknown = unknown_fields.UnknownFieldSet(d)
        if len(unknown):
            m["campos_desconhecidos"] = [f.field_number for f in unknown]
    except Exception:  # noqa: BLE001
        pass
    return m
