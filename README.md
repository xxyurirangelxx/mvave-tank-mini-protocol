# M-VAVE Tank Mini — Protocolo de Comunicação & Engenharia Reversa (MIDI & BLE)

[![Device](https://img.shields.io/badge/Hardware-M--VAVE%20Tank%20Mini-orange.svg)](https://www.m-vave.com)
[![Protocol](https://img.shields.io/badge/Protocol-BLE%20GATT%20%2F%20USB--MIDI-brightgreen.svg)]()
[![License](https://img.shields.io/badge/license-MIT-blue.svg)]()

> **Documento de Referência Aberta & Engenharia Reversa**  
> Autor: **Yuri Rangel** ([@xxyurirangelxx](https://github.com/xxyurirangelxx))  
> Firmware testado: `TANK MINI_009`

Este repositório contém a documentação técnica e engenharia reversa completa do protocolo de comunicação da pedaleira multi-efeitos **M-VAVE Tank Mini**. O objetivo deste documento é servir como especificação aberta para que desenvolvedores possam criar seus próprios softwares, aplicativos móveis, controladores físicos (ESP32, Arduino, Raspberry Pi) ou integrações DAW.

---

## 1. Visão Geral do Hardware e Transporte

A Tank Mini suporta dois meios de comunicação:
1. **Bluetooth Low Energy sem fio (BLE / GATT proprietário)** — *Recomendado*
2. **USB com fio (USB-MIDI Class-Compliant)**

### 1.1 Comunicação sem fio (Bluetooth BLE)
* **Nome do Dispositivo (Advertisement):** `TANK MINI_BLE`
* **Endereço MAC padrão:** Formato OUI MediaTek/M-VAVE (exemplo: `5C:76:0D:2A:9C:4F`)
* **Serviço GATT Proprietário:** `0000ae40-0000-1000-8000-00805f9b34fb` (short `ae40`)
* **Característica TX (Envio de Comandos):** `0000ae41-0000-1000-8000-00805f9b34fb` (short `ae41`)
  * Propriedade: `Write Without Response`
* **Característica RX (Notificações / Respostas):** `0000ae42-0000-1000-8000-00805f9b34fb` (short `ae42`)
  * Propriedade: `Notify`
* **Formato dos Dados no BLE:** **Bytes puros de 8 bits (Raw)**. No BLE **NÃO** se utiliza encapsulamento SysEx (`F0`/`F7`) nem empacotamento Base-128. Os pacotes binários descritos nas seções abaixo são transmitidos diretamente.

### 1.2 Comunicação via Cabo USB (USB MIDI)
* **Nome do Dispositivo:** `USB-Midi`
* **Padrão:** MIDI Class-Compliant 1.0 (WinMM / CoreMIDI / ALSA / Web MIDI API).
* **Restrição de 7 bits:** O protocolo universal MIDI proíbe bytes $\ge 0\text{x}80$ em dados comuns. Por isso, para USB a M-VAVE empacota o payload bruto de 8 bits em **Base-128 (7-bit bit-packing contínuo)** envolvido por marcadores SysEx (`0xF0` início, `0xF7` fim).

#### Algoritmo de Empacotamento SysEx (Base-128):
```python
def encode_sysex(source: bytes) -> bytes:
    accum = 0
    bit_num = 0
    dest = bytearray()
    for b in source:
        accum |= (b << (bit_num & 0x1F))
        bit_num += 1
        while True:
            dest.append(accum & 0x7F)
            accum >>= 7
            if bit_num < 7:
                break
            else:
                bit_num -= 7
    dest.append(accum & 0x7F)
    return bytes([0xF0] + list(dest) + [0xF7])

def decode_sysex(sysex: bytes) -> bytes:
    if len(sysex) < 2 or sysex[0] != 0xF0 or sysex[-1] != 0xF7:
        return b""
    body = sysex[1:-1]
    dest = bytearray()
    bit_num = 0
    last_val = 0
    for b in body:
        mask = (1 << bit_num) - 1 if bit_num > 0 else 0
        accum = b & mask
        if bit_num > 0:
            dest.append(((last_val & 0x7F) | ((accum << (8 - bit_num)) & 0xFF)) & 0xFF)
        last_val = b >> bit_num
        bit_num = (bit_num + 1) % 8
    if last_val > 0:
        dest.append(last_val)
    return bytes(dest)
```

---

## 2. Estrutura Básica de Pacotes M-VAVE (8-bit Raw)

Todos os comandos seguem uma estrutura padronizada de $N$ bytes:

| Offset | Tamanho | Função | Descrição |
|:---:|:---:|:---|:---|
| `0` | 2 bytes | **Header M-VAVE** | Sempre `0x00 0x59` |
| `2` | 1 byte | **Tipo de Operação** | `0x22` = Escrita (RAM), `0x23` = Leitura (RAM/Flash), `0x00` = ACK |
| `3` | 1 byte | **Comando / Tamanho** | `0x09` (Toggle/Preset/Modelo), `0x0A` (Knob), `0x0E` (Routing), `0x08` (Query/Dump Req), `0x64` (Full Dump 100B) |
| `4` | 3 bytes | **Roteamento** | Padrão: `0x00 0x00 0x04` (ou `0x00 0x00 0x02` para Routing) |
| `7` | 1 byte | **Endereço Alvo** | ID do bloco (`0x08`..`0x0D`) ou ID do preset (`0x00`..`0x14`) |
| `8` | 2 bytes | **Padding** | Normalmente `0x00 0x00` |
| `10` | 2 bytes | **Sub-comando** | `0x10 0x01` (Toggle/Modelo), `0x10 0x02` (Knob), `0x10 0x06` (Routing), `0xE0 0x01` (Preset) |
| `12` | Variável | **Dados / Valor** | `0x00 0x00 [VALOR]` |
| `N-1` | 1 byte | **Checksum** | Byte de integridade |

### 2.1 Fórmula Universal de Checksum
Calculado a partir do offset 4 até o penúltimo byte ($N-2$):

$$\text{Checksum} = 0\text{xFF} - \left(\sum_{i=4}^{N-2} \text{byte}[i] \pmod{256}\right)$$

### 2.2 Resposta de Confirmação (ACK) da Pedaleira
Sempre que um comando de escrita (Preset, Toggle, Knob, Routing) é executado com sucesso, a pedaleira devolve imediatamente via BLE Notify (`ae42`) ou USB SysEx o pacote de 8 bytes:
```text
00 59 00 01 00 00 00 FF
```

---

## 3. Mapeamento de Presets (1A até 7C)

A Tank Mini possui **21 presets físicos**, divididos em 7 bancos com 3 slots cada (A, B e C).

### 3.1 Fórmula de Indexação
O índice interno é linear de base zero ($0$ a $20$):

$$\text{ID} = (\text{Banco} - 1) \times 3 + \text{Slot}$$

Onde $\text{Banco} \in \{1..7\}$ e $\text{Slot}: A=0, B=1, C=2$.

| Banco | Slot A | Slot B | Slot C |
|:---:|:---:|:---:|:---:|
| **1** | `1A` (ID 0 / `0x00`) | `1B` (ID 1 / `0x01`) | `1C` (ID 2 / `0x02`) |
| **2** | `2A` (ID 3 / `0x03`) | `2B` (ID 4 / `0x04`) | `2C` (ID 5 / `0x05`) |
| **3** | `3A` (ID 6 / `0x06`) | `3B` (ID 7 / `0x07`) | `3C` (ID 8 / `0x08`) |
| **4** | `4A` (ID 9 / `0x09`) | `4B` (ID 10 / `0x0A`) | `4C` (ID 11 / `0x0B`) |
| **5** | `5A` (ID 12 / `0x0C`) | `5B` (ID 13 / `0x0D`) | `5C` (ID 14 / `0x0E`) |
| **6** | `6A` (ID 15 / `0x0F`) | `6B` (ID 16 / `0x10`) | `6C` (ID 17 / `0x11`) |
| **7** | `7A` (ID 18 / `0x12`) | `7B` (ID 19 / `0x13`) | `7C` (ID 20 / `0x14`) |

### 3.2 Comandos de Troca de Preset
* **Padrão MIDI (USB):** `0xC0 [ID]` (ex: `C0 04` para 2B).
* **Pacote M-VAVE (BLE ou SysEx — 16 bytes):**
  ```text
  00 59 22 09 00 00 04 [PRESET_ID] 00 00 E0 01 00 00 01 [CS]
  ```

---

## 4. Remanejamento da Cadeia de Sinal (Routing dos Efeitos)

A pedaleira permite **reordenar os 6 blocos de efeito em qualquer ordem** que o músico desejar (por exemplo, colocar Delay ou Modulação antes do Pré-amplificador).

### 4.1 Identificadores dos Blocos no Roteamento
* `0x00`: **FX** (Noise Gate, Compressor, Boost)
* `0x01`: **AMP** (Pré-amplificador)
* `0x02`: **MOD** (Modulação)
* `0x03`: **DLY** (Delay)
* `0x04`: **REV** (Reverb)
* `0x05`: **CAB** (Gabinete / IR)

### 4.2 Comando de Troca de Roteamento (21 bytes)
```text
00 59 22 0E 00 00 04 02 00 00 10 06 00 00 [B1] [B2] [B3] [B4] [B5] [B6] D4
```
* `[B1]` até `[B6]`: Sequência desejada dos blocos (`0x00` a `0x05`).
* **Nota de Checksum:** Como qualquer permutação dos 6 blocos $\{0, 1, 2, 3, 4, 5\}$ sempre soma $15$ ($0\text{x0F}$), o byte de Checksum para qualquer rota válida é **sempre fixo em `0xD4`**!

**Exemplos Práticos:**
* **Padrão de Fábrica (FX $\rightarrow$ AMP $\rightarrow$ MOD $\rightarrow$ DLY $\rightarrow$ REV $\rightarrow$ CAB):**
  `00 59 22 0E 00 00 04 02 00 00 10 06 00 00 00 01 02 03 04 05 D4`
* **Modulação antes do Amp (Phaser/Univibe vintage estilo Hendrix):**
  `00 59 22 0E 00 00 04 02 00 00 10 06 00 00 00 02 01 03 04 05 D4`
* **Delay antes do Amp (Ecos sujos e saturados no estilo Shoegaze/Post-Rock):**
  `00 59 22 0E 00 00 04 02 00 00 10 06 00 00 00 03 01 02 04 05 D4`
* **Reverb antes do Delay (Texturas densas e ambientais):**
  `00 59 22 0E 00 00 04 02 00 00 10 06 00 00 00 01 02 04 03 05 D4`

---

## 5. Mapeamento e Configuração de Cada Módulo

### 5.1 Endereços de Controle do Bloco
Cada bloco possui um endereço para **Toggle (Ligar/Bypass)**, um endereço para **Seleção de Modelo** e um endereço inicial para seus **Knobs/Parâmetros**:

| Bloco | Toggle Addr | Model Addr | Knobs Base Addr | Quantidade de Knobs |
|:---:|:---:|:---:|:---:|:---:|
| **FX** | `0x08` | `0x0E` | `0x14` | Até 4 |
| **AMP** | `0x09` | `0x0F` | `0x20` | 5 |
| **MOD** | `0x0A` | `0x10` | `0x2C` | Até 5 |
| **DLY** | `0x0B` | `0x11` | `0x38` | Até 6 |
| **REV** | `0x0C` | `0x12` | `0x44` | Até 5 |
| **CAB** | `0x0D` | `0x13` | `0x50` | 3 |

* **Comando de Toggle (Ligar/Desligar — 16 bytes):**
  `00 59 22 09 00 00 04 [TOGGLE_ADDR] 00 00 10 01 00 00 [01 ou 00] [CS]`
* **Comando de Troca de Modelo (16 bytes):**
  `00 59 22 09 00 00 04 [MODEL_ADDR] 00 00 10 01 00 00 [MODEL_ID] [CS]`
* **Comando de Ajuste de Knob (17 bytes):**
  `00 59 22 0A 00 00 04 [KNOB_ADDR] 00 00 10 02 00 00 [VAL_HEX] 00 [CS]`  
  *(Cada parâmetro ocupa 2 bytes na memória em Little-Endian, avançando de 2 em 2: `0x38`, `0x3A`, `0x3C`... Valores de 0 a 100 decimal).*

---

### 5.2 Módulo DELAY (DLY)
* **Toggle:** `0x0B` | **Model Addr:** `0x11` | **Knobs:** Início em `0x38`

#### Modelos Disponíveis:
* `0`: **Analog** — Emulação de delay de linha de retardo analógico (BBD), com repetições quentes e filtro de corte de agudos.
* `1`: **Duck (Ducking Delay)** — Inclui um gate dinâmico no rastro molhado que atenua as repetições enquanto a guitarra está tocando e solta os ecos nas pausas.
* `2`: **Dtape (Tape Echo)** — Máquina de fita magnética com saturação de pré-amp, irregularidades físicas de tração e modulação de fita.
* `3`: **Dual** — Dois ecos independentes com defasagem de tempo para padrões rítmicos estéreo.
* `4`: **Lofi** — Repetições propositalmente degradadas com granulação digital, chiado e filtro retro.

#### Parâmetros por Modelo de Delay:
| Parâmetro | Endereço | Descrição | Modelos Compatíveis |
|---|:---:|---|---|
| **Time** | `0x38` | Tempo do delay (60–600 bpm / 100ms–1000ms nos modelos 0,1,2,4; 120–1200 bpm / 50ms–500ms no Dual) | Todos |
| **Feedback** | `0x3A` | Quantidade de repetições (0 a 100%) | Todos |
| **Mix** | `0x3C` | Volume das repetições em relação ao som limpo (0 a 100%) | Todos |
| **Phaser Amount** | `0x3E` | Intensidade do phaser adicionado aos ecos | Somente `0 (Analog)` |
| **Phaser Freq** | `0x40` | Velocidade da oscilação do phaser | Somente `0 (Analog)` |
| **Unpack** | `0x3E` | Curva de subida de volume pós-palhetada para suavizar o efeito ducking | Somente `1 (Duck)` |
| **Tape Grit / Bias** | `0x3E` | Polarização da cabeça de fita magnética (saturação/overdrive no eco) | Somente `2 (Dtape)` e `4 (Lofi)` |
| **Tape Crinkle** | `0x40` | Simulação de vincos, poeira e atrito na fita | Somente `2 (Dtape)` |
| **Tape Flutter / Depth** | `0x42` | Flutuações mecânicas da velocidade da fita (modulação do pitch) | Somente `2 (Dtape)` |
| **Time-Mode / Lag** | `0x3E` | Diferença de tempo entre o primeiro e o segundo eco | Somente `3 (Dual)` |
| **Chorus Speed** | `0x40` | Velocidade da modulação de chorus no rastro | `1 (Duck)`, `3 (Dual)`, `4 (Lofi)` |
| **Chorus Depth** | `0x42` | Profundidade da modulação de chorus no rastro | `1 (Duck)`, `3 (Dual)`, `4 (Lofi)` |

---

### 5.3 Módulo REVERB (REV)
* **Toggle:** `0x0C` | **Model Addr:** `0x12` | **Knobs:** Início em `0x44`

#### Modelos Disponíveis:
* `0`: **Room** — Simula uma sala pequena com absorção e reflexões curtas de parede.
* `1`: **Hall** — Salão nobre de grande porte, com difusão ampla e decaimento longo.
* `2`: **Swell** — O reverb entra gradualmente atrás da nota palhetada (efeito volume swell / violino).
* `3`: **Spring** — Tanque de molas mecânicas clássico vintage.
* `4`: **Shimmer** — Adiciona cauda oitavada superior etérea e brilhante.
* `5`: **Cloud** — Reverb ambiente denso, suave e atmosférico.

#### Parâmetros por Modelo de Reverb:
| Parâmetro | Endereço | Descrição | Modelos Compatíveis |
|---|:---:|---|---|
| **Decay** | `0x44` | Duração do decaimento das reverberações | Todos |
| **Mix** | `0x46` | Nível de sinal molhado (wet/dry) | Todos |
| **HPass** | `0x48` | Filtro Passa-Altas: remove embolamento dos graves no rastro | `0`, `1`, `2`, `3`, `5` |
| **LPass** | `0x4A` | Filtro Passa-Baixas: atenua agudos estridentes na cauda | `0`, `1`, `2`, `3`, `5` |
| **Depth** | `0x4C` | Pequena ondulação cíclica de pitch vintage no fundo | `0 (Room)` e `1 (Hall)` |
| **RiseTime** | `0x4C` | Tempo que o efeito leva para atingir volume máximo após a palhetada | Somente `2 (Swell)` |
| **Combs** | `0x4C` | Tensão e quantidade de molas virtuais | Somente `3 (Spring)` |
| **Tone** | `0x48` | Resposta de frequências altas do efeito shimmer | Somente `4 (Shimmer)` |
| **Pitch** | `0x4A` | Intervalo do pitch shifter (de uníssono até +2 oitavas) | Somente `4 (Shimmer)` |
| **Amount** | `0x4C` | Intensidade do sinal oitavado na cauda | Somente `4 (Shimmer)` |
| **Diffusion** | `0x4C` | Espalhamento das reflexões iniciais para suavizar o impacto | Somente `5 (Cloud)` |

---

### 5.4 Módulo MODULAÇÃO (MOD)
* **Toggle:** `0x0A` | **Model Addr:** `0x10` | **Knobs:** Início em `0x2C`

#### Modelos Disponíveis:
* `0`: **Chorus** — Modulação de afinação clássica suave e encorpada.
* `1`: **Phaser** — Filtro de etapas de fase vintage.
* `2`: **Tremolo** — Variação cíclica de volume por LFO.
* `3`: **Flanger** — Linha curta de atraso com alta realimentação (efeito turbina).
* `4`: **Vibrato** — Modulação pura de afinação sem sinal direto misturado.
* `5`: **Univibe** — Clássica modulação optoeletrônica psicodélica dos anos 60.
* `6`: **Autofilter** — Auto-wah dinâmico baseado em envelope de palhetada.

#### Parâmetros por Modelo de Modulação:
| Parâmetro | Endereço | Descrição | Modelos |
|---|:---:|---|---|
| **Speed** | `0x2C` | Velocidade / Taxa de oscilação do LFO | Todos |
| **Depth** | `0x2E` | Profundidade / Intensidade da modulação | `0, 2, 3, 4, 5` |
| **Mix** | `0x30` | Proporção wet/dry | `0, 3, 5, 6` |
| **Dry** | `0x2E` | Volume do sinal seco no Phaser | Somente `1 (Phaser)` |
| **Wet / Reso** | `0x30` | Ressonância das etapas do filtro | Somente `1 (Phaser)` |
| **Feedback** | `0x32` | Regeneração das etapas de fase/flange | `1 (Phaser)`, `3 (Flanger)`, `6 (Autofilter)` |
| **Level** | `0x30` | Compensação de ganho de saída | Somente `2 (Tremolo)` |
| **Min Freq** | `0x2E` | Frequência inferior da varredura do filtro | Somente `6 (Autofilter)` |
| **Max Freq** | `0x30` | Frequência superior da varredura do filtro | Somente `6 (Autofilter)` |

---

### 5.5 Módulo FX (Drives, Gates e Compressores)
* **Toggle:** `0x08` | **Model Addr:** `0x0E` | **Knobs:** Início em `0x14`

#### Modelos Disponíveis:
* `0`: **Noise Gate++** — Redutor de ruído de alta precisão para timbres de alto ganho.
* `1`: **Boost** — Ganho limpo transparente para empurrar a entrada do amplificador.
* `2`: **Compress** — Compressor óptico/estúdio para controle de dinâmica e sustain.
* `3`: **AI Gate MS** — Gate inteligente com rastreamento neural para preservar sustentação de notas.

#### Parâmetros:
| Parâmetro | Endereço | Descrição | Modelos |
|---|:---:|---|---|
| **Gate Threshold** | `0x14` | Ponto de corte do ruído (0 = desativado, 100 = corte agressivo) | Todos |
| **Boost Gain** | `0x16` | Quantidade de ganho limpo injetado | Somente `1 (Boost)` |
| **Comp Attack** | `0x18` | Tempo de reação à palhetada inicial | Somente `2 (Compress)` |
| **Comp Sustain** | `0x16` | Tempo de sustentação da compressão | Somente `2 (Compress)` |
| **Comp Level** | `0x1A` | Volume de saída compensado | Somente `2 (Compress)` |
| **AI Bias** | `0x16` | Polarização e sensibilidade do rastreamento de notas | Somente `3 (AI Gate MS)` |

---

### 5.6 Módulo AMP (Modelagem de Pré-Amplificador)
* **Toggle:** `0x09` | **Model Addr:** `0x0F` | **Knobs:** Início em `0x20`

Todos os 20 modelos compartilham o mesmo conjunto de 5 parâmetros:
* `0x20`: **Gain** — Nível de saturação/drive do pré-amplificador (0 a 100)
* `0x22`: **Level** — Volume geral do bloco (0 a 100)
* `0x24`: **Bass** — Equalização de frequências graves (0 a 100)
* `0x26`: **Mid** — Equalização de frequências médias (0 a 100)
* `0x28`: **Treble** — Equalização de frequências agudas (0 a 100)

#### Lista Completa dos 20 Modelos Nativos do Firmware:
| ID | Código Interno | Amplificador de Referência | Estilo |
|:---:|---|---|---|
| `0` | `1J120_CL` | Roland JC-120 Jazz Chorus | Clean cristalino estéreo |
| `1` | `2J900_CL` | Marshall JCM900 Clean | Clean britânico dinâmico |
| `2` | `3VAVE_TS` | Ibanez Tube Screamer Preamp | Overdrive com médios focados |
| `3` | `4RAT_OD` | ProCo RAT | Overdrive rasgado/vintage |
| `4` | `5VAVE_HTS` | Custom High-Gain TS | Drive encorpado |
| `5` | `6VAVE_DS` | Boss DS-1 Distortion | Distorção clássica |
| `6` | `7MS_GV2` | Marshall The Guv'nor | Drive britânico pesado |
| `7` | `8J900_HV` | Marshall JCM900 Lead | Hard Rock britânico |
| `8` | `9MS_HIGAIN` | Marshall Modern High Gain | Rock moderno |
| `9` | `10J900-FG` | Marshall JCM900 Full Gain | Alto ganho clássico |
| `10` | `11.410H-FG` | Marshall JVM410H High Gain | Metal moderno britânico |
| `11` | `12MES-FG` | Mesa Boogie Dual Rectifier | Metal americano moderno |
| `12` | `13MES-_CH3` | Mesa Boogie Triple Rectifier Canal 3 | Graves densos e corte afiado |
| `13` | `14FIRMAN` | Friedman BE-100 Brown Eye | High-gain boutique |
| `14` | `15CT-Gas V4H` | Diezel VH4 Lead | Metal progressivo/pesado |
| `15` | `16AgDb750_BS` | Aguilar DB750 | Cabeçote de Contrabaixo moderno |
| `16` | `17ApSVT_BS` | Ampeg SVT-CL | Cabeçote de Contrabaixo valvulado clássico |
| `17` | `DS` | Distorção genérica alternativa | Rock |
| `18` | `OVERDRIVE` | Overdrive genérico | Blues/Pop |
| `19` | `CRUNCH` | Crunch leve | Rock clássico |

---

### 5.7 Módulo CAB (Impulse Response de Caixa Acústica)
* **Toggle:** `0x0D` | **Model Addr:** `0x13` | **Knobs:** Início em `0x50`

Todos os 20 modelos de IR compartilham os mesmos 3 controles:
* `0x50`: **Level** — Volume da simulação de caixa (0 a 100)
* `0x52`: **Low Cut** — Filtro passa-altas para cortar subgraves indesejados (20Hz a 300Hz, valor 0 a 100)
* `0x54`: **High Cut** — Filtro passa-baixas para suavizar chiados e agudos ásperos (18kHz a 5kHz, valor 0 a 100)

#### Lista Completa dos 20 Gabinetes Nativos do Firmware:
| ID | Código Interno | Caixa e Microfone Emulados | Instrumento |
|:---:|---|---|---|
| `0` | `1AC-SeVin` | Custom 2x12 Vintage | Guitarra |
| `1` | `2JVM_1960_57` | Marshall 1960A 4x12 (Shure SM57) | Guitarra |
| `2` | `3MESA_412` | Mesa Boogie Rectifier Oversized 4x12 | Guitarra |
| `3` | `4WANGS112` | Wangs 1x12 Custom | Guitarra |
| `4` | `5VOX_AC30` | Vox AC30 2x12 Alnico Blue | Guitarra |
| `5` | `6HIFI_YAMAHA` | Resposta plana Hi-Fi / Full Range | Violão / Linha |
| `6` | `7EGNL_412` | Engl Pro 4x12 Celestion V30 | Guitarra |
| `7` | `8V30_MD421` | 4x12 Celestion V30 (Sennheiser MD421) | Guitarra |
| `8` | `9Peavey115B` | Peavey 1x15 Bass Enclosure | Contrabaixo |
| `9` | `10OrPPC212` | Orange PPC212 Closed Back | Guitarra |
| `10` | `11TraceElliot` | Trace Elliot 4x10 Bass Cab | Contrabaixo |
| `11` | `12Bog2VTY` | Bogner 2x12 Vintage 30 | Guitarra |
| `12` | `13BGN412` | Bogner Ubercab 4x12 (V30 + T75) | Guitarra |
| `13` | `14EV MIX` | EVM-12L Custom Mix | Guitarra |
| `14` | `15Friedman` | Friedman 4x12 Vintage 30 | Guitarra |
| `15` | `16Agula410` | Aguilar GS410 | Contrabaixo |
| `16` | `17AgSVT410` | Ampeg SVT-410HLF | Contrabaixo |
| `17` | `18DgDG212N` | Darkglass 2x12 Neodymium | Contrabaixo |
| `18` | `19MbSway210` | Markbass Standard 2x10 | Contrabaixo |
| `19` | `20Tace412` | Vintage 4x12 Classic | Guitarra |

---

## 6. Estrutura da Matriz RAM (Dump Completo de 92 Bytes)

Para capturar ou restaurar o estado integral de todos os efeitos e roteamento em uma única operação:

* **Requisição:** `00 59 23 08 00 00 04 00 00 00 10 5C 00 00 8F` (16 bytes)
* **Resposta:** `00 59 23 64 00 00 04 00 00 00 10 5C 00 00 [92 BYTES] [CS]` (107 bytes)

### Mapa Exato dos 92 Bytes:
```
Offset 0..5   : Sequência de roteamento [B1, B2, B3, B4, B5, B6] (0=FX, 1=AMP, 2=MOD, 3=DLY, 4=REV, 5=CAB)
Offset 6..7   : Padding interno de alinhamento
Offset 8      : FX Ativado (0x01 = Ligado, 0x00 = Bypass)
Offset 9      : AMP Ativado (0x01 = Ligado, 0x00 = Bypass)
Offset 10     : MOD Ativado (0x01 = Ligado, 0x00 = Bypass)
Offset 11     : DLY Ativado (0x01 = Ligado, 0x00 = Bypass)
Offset 12     : REV Ativado (0x01 = Ligado, 0x00 = Bypass)
Offset 13     : CAB Ativado (0x01 = Ligado, 0x00 = Bypass)
Offset 14     : Modelo ativo do FX (0 a 3)
Offset 15     : Modelo ativo do AMP (0 a 19)
Offset 16     : Modelo ativo do MOD (0 a 6)
Offset 17     : Modelo ativo do DLY (0 a 4)
Offset 18     : Modelo ativo do REV (0 a 5)
Offset 19     : Modelo ativo do CAB (0 a 19)
Offset 20..31 : Knobs FX (cada knob ocupa 2 bytes Little-Endian: valor + 0x00)
Offset 32..43 : Knobs AMP (Gain=32, Level=34, Bass=36, Mid=38, Treble=40)
Offset 44..55 : Knobs MOD (Speed=44, Depth=46, Mix=48...)
Offset 56..67 : Knobs DLY (Time=56, Feedback=58, Mix=60, Param4=62, Param5=64, Param6=66)
Offset 68..79 : Knobs REV (Decay=68, Mix=70, HPass=72, LPass=74, Param5=76)
Offset 80..91 : Knobs CAB (Level=80, LowCut=82, HighCut=84)
```

---

## 7. Gravação Permanente na Memória Flash (Save)

Quando você altera parâmetros com `0x22`, a mudança ocorre instantaneamente na **RAM** (o som muda em tempo real). Para tornar a alteração **permanente** (resistindo a desligar da tomada ou trocar de preset):

1. Obtenha a matriz atualizada de 92 bytes.
2. Calcule o endereço físico da Flash do preset:
   $$\text{FlashAddress} = \text{PresetID} \times 92$$
   *(ID de 0 a 20; injetado como inteiro Little-Endian de 4 bytes).*
3. Monte o pacote de 107 bytes de gravação DMA:
   ```text
   Header (14 bytes) : 00 59 22 64 00 00 04 [ADDR_L] [ADDR_H] 00 00 5C 00 00
   Payload (92 bytes): [92 BYTES DA MATRIZ]
   Checksum (1 byte) : [CHECKSUM CALCULADO DO PACOTE DE 107 BYTES]
   ```
4. Ao receber o pacote, a pedaleira grava na memória flash e responde com o ACK:
   `00 59 00 01 00 00 00 FF`.
