# SPDX-License-Identifier: GPL-3.0-or-later
"""Ligação com o módulo Meshtastic via USB (serial): chaves, nodedb e pacotes recebidos."""
from __future__ import annotations

import queue
import time

from mesh_decode import (PRESET_ENUM_TO_NAME, ChannelKey, KeyRing, NodeDB, expand_psk)


class ModuleError(Exception):
    pass


USB_CHIPS = {0x1A86: "WCH CH340/CH9102", 0x10C4: "Silicon Labs CP210x", 0x0403: "FTDI",
             0x303A: "Espressif (USB nativo ESP32-S3/C3)", 0x239A: "Adafruit/nRF52 (RAK, T-Echo)",
             0x2886: "Seeed (XIAO/T1000)", 0x2E8A: "Raspberry Pi RP2040"}


def list_serial_ports() -> list[str]:
    import serial.tools.list_ports
    out = []
    for p in serial.tools.list_ports.comports():
        chip = USB_CHIPS.get(p.vid, "") if p.vid else ""
        vidpid = f"VID:PID {p.vid:04X}:{p.pid:04X}" if p.vid else "sem USB"
        out.append(f"{p.device}  {p.description}  ({vidpid}{', ' + chip if chip else ''})")
    return out


def find_module_port() -> str:
    from meshtastic.util import findPorts
    ports = findPorts(True)
    if len(ports) == 1:
        return ports[0]
    lst = list_serial_ports()
    if not ports:
        msg = "nenhuma porta serial USB encontrada."
        if lst:
            msg += " Portas existentes (nenhuma parece ser o módulo):\n      " + "\n      ".join(lst)
        msg += ("\n    Verifique: o cabo USB transmite dados (muitos cabos são só de carga)?"
                "\n    O driver do chip USB do módulo está instalado (CH340/CH9102 ou CP210x)?"
                "\n    No Gerenciador de Dispositivos, em 'Portas (COM e LPT)', aparece algo ao plugar o módulo?")
        raise ModuleError(msg)
    raise ModuleError("mais de uma porta possível; escolha com --porta:\n      " + "\n      ".join(lst))


class ModuleLink:
    def __init__(self, port: str | None = None):
        import meshtastic.serial_interface
        from pubsub import pub

        self.events: queue.Queue = queue.Queue()
        if not port:
            port = find_module_port()
        pub.subscribe(self._on_rx, "meshtastic.receive")
        pub.subscribe(self._on_lost, "meshtastic.connection.lost")
        self.iface = meshtastic.serial_interface.SerialInterface(devPath=port)
        if not hasattr(self.iface, "localNode") or self.iface.localNode is None:
            raise ModuleError(f"a porta {port} abriu mas o módulo não respondeu ao protocolo Meshtastic")
        self.port = port

    # ------------------------------------------------------------ eventos
    def _on_rx(self, packet, interface=None):  # noqa: ARG002
        self.events.put(("rx", time.time(), packet))

    def _on_lost(self, interface=None):  # noqa: ARG002
        self.events.put(("lost", time.time(), None))

    # ------------------------------------------------------------ informações
    def my_num(self) -> int | None:
        try:
            return int(self.iface.myInfo.my_node_num)
        except Exception:  # noqa: BLE001
            return None

    def lora_config(self) -> dict:
        try:
            from google.protobuf.json_format import MessageToDict
            return MessageToDict(self.iface.localNode.localConfig.lora, preserving_proto_field_name=True)
        except Exception:  # noqa: BLE001
            return {}

    def preset_name(self) -> str:
        lc = self.lora_config()
        if lc.get("use_preset", True) is False:
            return "Custom"
        return PRESET_ENUM_TO_NAME.get(lc.get("modem_preset", "LONG_FAST"), "LongFast")

    def load_keys(self, ring: KeyRing) -> list[ChannelKey]:
        from meshtastic.protobuf import channel_pb2
        added = []
        chans = list(self.iface.localNode.channels or [])
        preset = self.preset_name()
        primary_key = b""
        for ch in chans:
            if ch.role == channel_pb2.Channel.Role.PRIMARY:
                primary_key = expand_psk(bytes(ch.settings.psk))
        for ch in chans:
            if ch.role == channel_pb2.Channel.Role.DISABLED:
                continue
            name = ch.settings.name or (preset if ch.role == channel_pb2.Channel.Role.PRIMARY else "")
            if not name:
                name = preset
            psk = bytes(ch.settings.psk)
            key = expand_psk(psk)
            if not psk and ch.role == channel_pb2.Channel.Role.SECONDARY:
                key = primary_key
            aead = bool(getattr(ch.settings, "use_aead", False))
            ck = ChannelKey(name, key, aead, "módulo", ch.index)
            ring.add(ck)
            added.append(ck)
        try:
            pk = bytes(self.iface.localNode.localConfig.security.private_key)
            if len(pk) == 32:
                ring.private_key = pk
        except Exception:  # noqa: BLE001
            pass
        return added

    def load_nodes(self, db: NodeDB) -> int:
        n = 0
        for _, info in (self.iface.nodes or {}).items():
            num = info.get("num")
            if num is None:
                continue
            user = info.get("user", {})
            db.update_user(int(num), {
                "long_name": user.get("longName", ""), "short_name": user.get("shortName", ""),
                "hw_model": user.get("hwModel", ""), "role": user.get("role", ""),
                "public_key": user.get("publicKey", ""),
            }, "módulo")
            pos = info.get("position") or {}
            if "latitude" in pos and "longitude" in pos:
                db.get(int(num)).position = (pos["latitude"], pos["longitude"])
            n += 1
        db.my_num = self.my_num()
        return n

    def close(self):
        try:
            self.iface.close()
        except Exception:  # noqa: BLE001
            pass
