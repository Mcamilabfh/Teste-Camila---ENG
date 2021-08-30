# Teste-Camila---ENG

## Pré requisitos:
 
  - Docklight
  - ESP-IDF

Instalação ESP-IDF [espressif](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/)

## GPIO

|    FUN    |    GPIO     |
|-----------|-------------|
|  TXD_PIN  |  GPIO_NUM_1 |
|  RXD_PIN  |  GPIO_NUM_3 |
|  GPIO_LED |  GPIO_NUM_2 |

## Wifi

|       FUN       |      GPIO       |
|-----------------|-----------------|
|  ESP_WIFI_SSID  |  ENG TECNOLOGIA |
|  ESP_WIFI_PASS  |     eng12345    |

## Protocolo **Sem** parâmetros

|  EN  |  CMD  | N PARAMETRO | CHECKSUM |  DESCRIÇÃO  |
|------|-------|-------------|----------|-------------|
| 0x01 | 0x00  |     0x00    |   0x01   |  Ligar led  |
| 0x01 | 0x01  |     0x00    |   0x02   | desligar led|
| 0x01 | 0x02  |     0x00    |   0x03   |  toggle led |
| 0x01 | 0x03  |     0x00    |   0x04   |  loopback   |

## Protocolo **Com** parâmetros

|  EN  |  CMD  | N PARAMETRO |PARAMETRO 1| PARAMETRO 2 | CHECKSUM |  DESCRIÇÃO  |
|------|-------|-------------|-----------|-------------|----------|-------------|
| 0x01 | 0x03  |     0x02    |    0x01   |     0x03    |   0x0A   |   loopback  |

Executando projeto:


    Abra o terminal do idf e execute os comandos relativos a sua máquina:

    cd ~/TESTE-CAMILA---ENG
    idf.py build
    idf.py -p COM3 flash


