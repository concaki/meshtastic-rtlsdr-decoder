# meshtastic-rtlsdr-decoder

*Read in English: [README.md](README.md)*

Decodificador passivo de pacotes Meshtastic para um **RTL-SDR** barato, em Python puro
(numpy/scipy, sem GNU Radio). Demodula o LoRa em software, decifra e decodifica cada pacote
Meshtastic no ar e cruza o resultado com o que o seu próprio nó (ligado por USB) recebeu.

Configuração padrão: **919,875 MHz, LongFast (SF11, 250 kHz, CR 4/5), região ANZ**. Qualquer
preset e frequência podem ser escolhidos na linha de comando.

## O que aparece para cada pacote

- **RF:** frequência real, desvio em Hz e ppm, SNR estimado, nível, tempo no ar
- **LoRa:** SF/BW/CR, sync word, cabeçalho, CRC, correções do FEC, deriva de frequência no pacote
- **Meshtastic:** origem e destino com nome do nó, id, saltos (hop_limit/hop_start), want_ack,
  via_mqtt, hash do canal, next_hop e relay_node (com o nó provável), retransmissões
- **Decifragem:** qual chave abriu — canal AES-CTR, canal AEAD (AES-CCM) ou PKI para mensagens
  diretas (só as DMs de/para o seu nó)
- **Conteúdo:** texto, posição (link de mapa e distância até o seu nó), telemetria, nodeinfo,
  traceroute com nomes e SNR por salto, ACK/NAK, vizinhos, waypoints… todos os campos do protobuf
- **Seu nó:** se ele também recebeu (RSSI/SNR do rádio) ou qual dos dois perdeu

Tudo é gravado também em `pacotes.jsonl` (uma linha JSON por pacote, com os bytes brutos).

### Robusto a dongles baratos

Ajustado com gravações reais de um dongle genérico R828D e coberto por testes:

- desvio de frequência grande (testado até ±60 kHz) e frequência variando no pacote (±4,2 kHz/s);
- relógio de amostragem do dongle mais de 100 ppm fora, independente do cristal do tuner;
- demodulação na taxa sobreamostrada, para erro fracionário de tempo não corromper símbolos;
- recuperação de um a três símbolos ambíguos com ajuda do CRC;
- leitura USB assíncrona, sem perder amostras entre blocos.

## Instalação

1. **Python 3.10 ou mais novo** (no Windows, marque "Add python.exe to PATH").
2. **Driver do RTL-SDR.** Windows: abra o Zadig, *Options → List All Devices*, escolha
   *Bulk-In, Interface (Interface 0)* e instale **WinUSB** (se o SDR# ou SDR++ já funcionam, já está
   feito). Linux/macOS: instale o `librtlsdr` (`apt install librtlsdr0` / `brew install librtlsdr`).
3. **Dependências:**

   ```
   pip install -r requirements.txt
   ```

4. *Só para o RTL-SDR Blog V4:* copie `rtlsdr.dll` e `libusb-1.0.dll` do driver Windows da RTL-SDR
   Blog para a pasta `drivers\`. Clones genéricos R828D funcionam com a DLL padrão.

## Uso

```
python mesh_sdr.py
python mesh_sdr.py --porta COM5 --ganho 10
```

No Windows, basta dar dois cliques em `iniciar.bat`.

O nó é detectado sozinho. Dele o programa importa canais e PSKs, a chave privada PKI (para
decifrar DMs de/para o seu nó) e a lista de nós com nomes e chaves públicas. A porta serial só
pode ser aberta por um programa de cada vez: feche o app Meshtastic antes.

| opção | para quê |
|---|---|
| `--porta COM5` | porta do seu nó (`--listar-portas` mostra as disponíveis) |
| `--sem-modulo` | só o SDR, sem o nó |
| `--ganho 10` / `--ganho auto` | ganho do RTL-SDR (padrão 0 dB; suba se o nó estiver longe) |
| `--ppm -52` | correção do cristal do dongle |
| `--freq 906.875 --preset MediumFast` | outra frequência / preset |
| `--canal "Nome:PSKbase64[:aead]"` | chave de canal extra (PSK como aparece no app) |
| `--chave-privada B64 --meu-no !a1b2c3d4` | decifrar suas DMs sem o nó ligado |
| `--gravar captura.cu8` | grava o IQ bruto enquanto decodifica |
| `--arquivo captura.cu8 --taxa 1000000` | decodifica uma gravação em vez do rádio ao vivo |
| `--diagnostico` | captura de 20 s em banda larga: ganho, saturação, rajadas no espectro, teste |
| `--mostrar-falhas` / `--hex` | mostra quadros com CRC ruim / bytes brutos |

Ajuda completa: `python mesh_sdr.py -h`.

### Primeira execução recomendada

```
python mesh_sdr.py --diagnostico
```

Mande duas ou três mensagens pelo seu nó durante a captura. O relatório mostra se o ganho foi
aplicado, a saturação do ADC, onde aparecem as rajadas e se decodificam, e sugere um `--ppm`.

## Testar sem rádio

```
python tools/gerar_sinal_teste.py teste.cu8     # gera uma gravação sintética no formato do rtl_sdr
python -m pytest -q                             # testes de PHY, criptografia e ponta a ponta (~1 min)
```

A gravação sintética tem 13 pacotes (texto, nodeinfo, posição, telemetria, canal privado, DM com
PKI, retransmissão, traceroute, ACK, canal AEAD, uma chave desconhecida e um a −10 dB de SNR).
No Windows, `testar_sem_radio.bat` gera e decodifica.

## Dicas de recepção

- O sinal fica a +250 kHz do centro sintonizado, longe do pico DC do dongle.
- Ganho padrão 0, porque o nó costuma ficar perto do SDR. Com saturação 0% e sinais fracos no
  diagnóstico, suba aos poucos (10, 20, 30 dB).
- A antena que vem com dongles de TV rende pouco em 915 MHz; uma antena de 915 MHz faz muita diferença.
- Pacotes que **só o nó** recebeu mostram onde o SDR está perdendo (antena, ganho, ruído).

## Arquivos

| arquivo | conteúdo |
|---|---|
| `mesh_sdr.py` | programa principal: terminal, log JSONL, cruzamento com o nó |
| `lora_phy.py` | demodulador LoRa: detecção, sincronismo, rastreio de CFO/tempo/deriva, FEC, CRC |
| `mesh_decode.py` | cabeçalho Meshtastic, AES-CTR/CCM, PKI X25519, protobufs |
| `module_link.py` | conexão serial com o nó via biblioteca `meshtastic` |
| `sources.py` | RTL-SDR (assíncrono), leitura de arquivos IQ, mistura/filtro/decimação |
| `diagnostico.py` | modo `--diagnostico` |
| `tools/gerar_sinal_teste.py` | gerador de sinal sintético |
| `tests/` | testes (pytest) |

## Privacidade e legislação

- Pacotes na chave padrão do LongFast são, na prática, públicos: qualquer um pode lê-los, com ou
  sem esta ferramenta. Canais privados e DMs só são decifrados com chaves que você fornece (as suas).
- Gravações (`*.cu8`) e logs (`*.jsonl`) podem conter ids, nomes, posições e mensagens de outras
  pessoas. Eles estão no `.gitignore`; pense antes de compartilhar.
- Receber sinais de rádio é permitido na maior parte dos lugares, mas confira as regras locais.

## Créditos

As convenções da camada física foram conferidas com o [gr-lora_sdr](https://github.com/tapparelj/gr-lora_sdr)
(Tapparel et al., EPFL) e o [LoRaPHY](https://github.com/jkadbear/LoRaPHY); o enquadramento, nonces,
PKI e canais AEAD do Meshtastic com o [firmware Meshtastic](https://github.com/meshtastic/firmware).
Nenhum código foi copiado desses projetos.

## Licença

[GPL-3.0](LICENSE)
