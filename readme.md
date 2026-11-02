# 🐾 HydraPet Vision v2.1

**HydraPet Vision** é um sistema embarcado inteligente desenvolvido para monitorar e registrar visitas de felinos a bebedouros. Utilizando a plataforma **ESP32-CAM**, o dispositivo detecta a presença do animal via sensor ultrassônico, registra a data e a hora com um RTC dedicado e captura fotografias (com suporte a visão noturna infravermelha), salvando os relatórios e imagens em um cartão MicroSD.

---

## 📸 Principais Funcionalidades

- **Conceito de Evento por "Visita":** Agrupa leituras contínuas. A visita inicia com a confirmação de presença e encerra após `VISITA_GAP_MS` sem detecção.
- **Visão Noturna Inteligente (IR):** Ativação automática dos LEDs infravermelhos baseada na leitura do sensor LDR em ambientes escuros.
- **Organização Estruturada em SD:** 
  - Fotos organizadas em subpastas por data (`/fotos/AAAA-MM-DD/`).
  - Relatório detalhado por fotografia (`/registros.csv`).
  - Relatório consolidado por visita (`/visitas.csv`).
- **Interface Gráfica em Display TFT (ILI9341):** Exibição de status em tempo real, visualização das fotos salvas, navegação por menus e ajuste de data/hora.
- **Resiliência e Autodiagnóstico:** Autoteste do sensor ultrassônico na inicialização, fallback de barramento I2C/SCCB e reconexão automática do cartão SD.
- **Compatibilidade:** Ficheiros CSV formatados com vírgula decimal para abertura direta no Microsoft Excel (PT-BR).

---

## 🛠️ Eschema e Mapa de Pinos (ESP32-CAM AI-Thinker)

### Mapeamento de GPIOs

| Pino ESP32-CAM | Periférico / Função | Observações |
| :--- | :--- | :--- |
| **GPIO 13** | TFT SCK / SD DAT3 | Modo SD 1-bit |
| **GPIO 12** | TFT MOSI (SDI) / SD DAT2 | Pino de boot (`MTDI`). **Não colocar pull-up!** |
| **GPIO 1** | TFT DC (`U0TXD`) | Serial desativada durante a operação |
| **GPIO 3** | HC-SR04 ECHO (`U0RXD`) | Requer divisor de tensão (1kΩ série / 2kΩ GND) |
| **GPIO 26** | I2C SDA | Compartilhado com SCCB da câmera (pull-up 4,7 kΩ para 3V3) |
| **GPIO 27** | I2C SCL | Compartilhado com SCCB da câmera (pull-up 4,7 kΩ para 3V3) |
| **GPIO 4** | LED Flash Interno | Mantido em `LOW` |
| **GPIO 0** | XCLK Câmera / Boot | Conectar ao `GND` apenas para gravação do firmware |

### Expansor de I/O PCF8574 (Endereço `0x20`)

- **P0:** Botão 1 (Avançar / Próximo)
- **P1:** Botão 2 (Confirmar / OK)
- **P2:** Botão 3 (Voltar / Retroceder)
- **P3:** HC-SR04 `TRIG`
- **P4:** Gate do MOSFET (LEDs IR)
- **P5:** Saída Digital do Módulo LDR (LM393)

---

## 💻 Requisitos e Dependências

### Bibliotecas Necessárias (Arduino IDE)

Instale as seguintes bibliotecas através do **Gerenciador de Bibliotecas**:
- `Adafruit GFX Library`
- `Adafruit ILI9341` (instala `Adafruit BusIO` automaticamente)
- `RTClib` (Adafruit)
- `TJpg_Decoder` (Bodmer)

*Bibliotecas Nativas do Núcleo ESP32:* `Wire`, `SPI`, `Preferences`, `FS`, `SD_MMC`, `esp_camera`.

### Configuração da Arduino IDE

- **Placa:** `AI Thinker ESP32-CAM`
- **Core Debug Level:** `None` *(Obrigatório: GPIO1 é utilizado pelo display)*
- **PSRAM:** `Enabled`
- **Partition Scheme:** `Huge APP (3MB No OTA/1MB SPIFFS)`

---

## 📁 Estrutura de Arquivos no Cartão MicroSD

```text
/
├── fotos/
│   └── AAAA-MM-DD/
│       ├── gato_20261102_143000.jpg
│       └── gato_20261102_143015.jpg
├── registros.csv       # data;hora;arquivo;distancia_cm;ir;visita
└── visitas.csv         # visita;data;hora_inicio;duracao_s;fotos
```

---

## 🚀 Como Gravar o Firmware

1. Conecte o pino `GPIO 0` ao `GND`.
2. Desconecte temporariamente os fios de `GPIO 1` e `GPIO 3`.
3. Conecte o módulo FTDI ao ESP32-CAM (TX -> RX, RX -> TX, 5V e GND).
4. Realize o upload via Arduino IDE.
5. Após o envio com sucesso, remova a ponte entre `GPIO 0` e `GND`, reconecte os pinos e reinicie a placa.

---

## 📄 Licença

Este projeto está licenciado sob a licença MIT - consulte o arquivo `LICENSE` para mais detalhes.