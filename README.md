# meshtastic-rtlsdr-decoder

*Leia em português: [README.pt-BR.md](README.pt-BR.md)*

Passive Meshtastic packet decoder for a cheap **RTL-SDR** dongle, written in pure Python
(numpy/scipy, no GNU Radio). It demodulates LoRa in software, decrypts and decodes every
Meshtastic packet on the air, and can cross-check the result against what your own Meshtastic
node (connected over USB) received.

Default setup: **919.875 MHz, LongFast (SF11, 250 kHz, CR 4/5), region ANZ**. Any preset and
frequency can be selected on the command line.

> The program's terminal output is in Portuguese (pt-BR). The code, options and log format are
> documented here in English.

## Features

For every packet heard:

- **RF:** real frequency, offset in Hz and ppm, estimated SNR, level, time on air
- **LoRa:** SF/BW/CR, sync word, header, payload CRC, FEC corrections, frequency drift during the packet
- **Meshtastic header:** from/to (with node names), packet id, hop_limit/hop_start, want_ack,
  via_mqtt, channel hash, next_hop and relay_node (with the likely node), retransmissions
- **Decryption:** which key worked — channel AES-CTR, AEAD channels (AES-CCM), or PKI direct
  messages (X25519 + AES-CCM, only for DMs to/from your own node)
- **Content:** text, position (map link and distance to your node), telemetry, node info,
  traceroute with names and per-hop SNR, ACK/NAK, neighbor info, waypoints… every protobuf field
- **Your node:** whether it also received the packet (its RSSI/SNR), or which side missed it

Everything is also written to `pacotes.jsonl` (one JSON object per packet, including raw bytes).

### Robust to cheap dongles

Tuned against real recordings from a generic R828D dongle and covered by tests:

- large frequency offsets (tested to ±60 kHz) and frequency drift during a packet (±4.2 kHz/s);
- ADC sample clock off by more than 100 ppm and independent from the tuner crystal;
- symbol demodulation at the oversampled rate, so fractional timing errors do not corrupt symbols;
- CRC-assisted recovery of one to three ambiguous symbols;
- asynchronous USB reading, so no samples are lost between blocks.

## Installation

1. **Python 3.10+** (on Windows tick "Add python.exe to PATH").
2. **RTL-SDR driver.** Windows: run Zadig, *Options → List All Devices*, pick
   *Bulk-In, Interface (Interface 0)* and install **WinUSB** (already done if SDR# or SDR++ work).
   Linux/macOS: install `librtlsdr` (e.g. `apt install librtlsdr0` / `brew install librtlsdr`).
3. **Dependencies:**

   ```
   pip install -r requirements.txt
   ```

4. *RTL-SDR Blog V4 only:* copy `rtlsdr.dll` and `libusb-1.0.dll` from the RTL-SDR Blog Windows
   driver into `drivers\`. Generic R828D clones work with the default DLL.

## Usage

```
python mesh_sdr.py                      # 919.875 MHz LongFast, node auto-detected
python mesh_sdr.py --porta COM5 --ganho 10   # option names are Portuguese, see the table
```

On Windows you can just double-click `iniciar.bat`.

Your node is detected automatically. From it the program imports channel names and PSKs, the PKI
private key (to decrypt DMs to/from your node) and the node list with names and public keys.
Only one program can open the serial port at a time — close the Meshtastic app first.

| option | meaning |
|---|---|
| `--porta COM5` | serial port of your node (`--listar-portas` lists them) |
| `--sem-modulo` | SDR only, no node |
| `--ganho 10` / `--ganho auto` | RTL-SDR gain (default 0 dB; raise it if the node is far away) |
| `--ppm -52` | dongle crystal correction |
| `--freq 906.875 --preset MediumFast` | other frequency / preset |
| `--canal "Name:PSKbase64[:aead]"` | extra channel key (PSK as shown in the app) |
| `--chave-privada B64 --meu-no !a1b2c3d4` | decrypt your DMs without the node attached |
| `--gravar capture.cu8` | record raw IQ while decoding |
| `--arquivo capture.cu8 --taxa 1000000` | decode a recording instead of the live dongle |
| `--diagnostico` | 20 s wideband capture: gain, clipping, bursts in the spectrum, decode test |
| `--mostrar-falhas` / `--hex` | show frames with bad CRC / raw bytes |

Full help: `python mesh_sdr.py -h`.

### Recommended first run

```
python mesh_sdr.py --diagnostico
```

Send two or three messages from your node during the capture. The report shows whether the gain
was applied, ADC clipping, where RF bursts appear and whether they decode, and suggests a `--ppm`.

## Testing without a radio

```
python tools/gerar_sinal_teste.py teste.cu8     # writes a synthetic rtl_sdr-style recording
python -m pytest -q                             # PHY, crypto and end-to-end tests (~1 min)
```

The synthetic recording holds 13 packets (text, node info, position, telemetry, private channel,
PKI DM, retransmission, traceroute, ACK, AEAD channel, one unknown key, one at −10 dB SNR).
On Windows, `testar_sem_radio.bat` generates and decodes it.

## Project layout

| file | contents |
|---|---|
| `mesh_sdr.py` | main program: terminal output, JSONL log, cross-check with the node |
| `lora_phy.py` | LoRa demodulator: detection, sync, CFO/timing/drift tracking, FEC, whitening, CRC |
| `mesh_decode.py` | Meshtastic header, AES-CTR/CCM, PKI X25519, protobuf decoding |
| `module_link.py` | serial link to the node via the `meshtastic` Python library |
| `sources.py` | RTL-SDR (async), IQ file reader, mixer/filter/decimator |
| `diagnostico.py` | `--diagnostico` mode |
| `tools/gerar_sinal_teste.py` | synthetic signal generator |
| `tests/` | pytest suite |

## Privacy and legal

- Packets on the default LongFast key are effectively public; anyone can read them, with or
  without this tool. Private channels and DMs are only decrypted with keys you provide (your own).
- Recordings (`*.cu8`) and logs (`*.jsonl`) can contain other people's node ids, names, positions
  and messages. They are git-ignored; think twice before sharing them.
- Receiving radio signals is legal in most places, but check your local rules.

## Credits

Physical-layer conventions were checked against [gr-lora_sdr](https://github.com/tapparelj/gr-lora_sdr)
(Tapparel et al., EPFL) and [LoRaPHY](https://github.com/jkadbear/LoRaPHY); Meshtastic framing,
nonces, PKI and AEAD channels against the [Meshtastic firmware](https://github.com/meshtastic/firmware).
No code was copied from those projects.

## License

[GPL-3.0](LICENSE)
