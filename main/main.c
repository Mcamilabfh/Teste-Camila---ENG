#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"


static const int BUF_SIZE = 1024;   ///< Buffer para Rx da UART Serial.

/**
 *  Estrutura para comunicação na fila responsável por enviar os bytes para o processo do tx_task.
 *  Usado como item da fila para comunicação com o tx_task.
 **/
typedef struct {
    uint8_t* data;  ///< Buffer de bytes a serem enviados pela Serial.
    int size;       ///< Tamanho em bytes do espaço ocupado pelos bytes no buffer data.
} send_event_t;


#define TXD_PIN (GPIO_NUM_1)                    ///< Pino que será usado para tx da UART.
#define RXD_PIN (GPIO_NUM_3)                    ///< Pino que será usado para rx da UART.
#define GPIO_LED (GPIO_NUM_2)                   ///< Pino que será usado para ligar ou desligar o led.
#define GPIO_OUTPUT_PIN_SEL (1ULL<<GPIO_LED)    ///< Máscara de bits para configuração do pino do led.

#define ESP_WIFI_SSID      "ENG TECNOLOGIA" ///< Definição do nome do ponto de acesso da wifi.
#define ESP_WIFI_PASS      "eng12345"       ///< Definição da senha do ponto de acesso da wifi.
#define ESP_WIFI_CHANNEL   10               ///< Definição do canal do ponto de acesso da wifi.
#define MAX_STA_CONN       100              ///< Definição da quantidade limite de usuários permitidos na conexão do ponto de acesso da wifi.

QueueHandle_t xQueue;    ///> Fila para comunicação da taks com a tarefa de transmissão.
QueueHandle_t xQueueLed; ///> Fila para comunicação da taks do led com a tarefa de transmissão do led.

static const char *TAG = "wifi ENG"; ///> Tag para indicação dos logs.

/**
 *  Indicação de usuários conectados e desconectados ao ponto de acesso da wifi.
 **/
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}
/**
 * Configurações gerais do wifi e ponto de acesso.
 **/
void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = ESP_WIFI_SSID,
            .ssid_len = strlen(ESP_WIFI_SSID),
            .channel = ESP_WIFI_CHANNEL,
            .password = ESP_WIFI_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init finished. SSID:%s password:%s channel:%d",
             ESP_WIFI_SSID, ESP_WIFI_PASS, ESP_WIFI_CHANNEL);
}
/**
 * Inicialização do wifi e uso do nvs para salvar informações básicas do wifi na memória flash caso o esp32 seja desernegizado.
 **/
void init_nvs(void) {
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

/**
 * Inicializa e configura a UART.
 **/
void init_uart(void) {
    const uart_config_t uart_config = {
            .baud_rate = 115200,                   ///< Taxa de transmissão de dados na velocidade do esp 115200.
            .data_bits = UART_DATA_8_BITS,         ///< Número de bits de dados = 8.               
            .parity = UART_PARITY_DISABLE,         ///< Bit de paridade desabilitado.
            .stop_bits = UART_STOP_BITS_1,         ///< Stop bit configurado com 1 bit.
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, ///< Controle de fluxo desabilitado.
            .source_clk = UART_SCLK_APB,           ///< Clock para o cálculo do baude rate.                        
    };
   
    uart_driver_install(UART_NUM_1, 2048, 0, 0, NULL, 0);                               ///< Instalação do driver da UART.
    uart_param_config(UART_NUM_1, &uart_config);                                        ///< Configuranção da UART.
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); ///< Set dos pinos tx e rx da UART.
}
/**
 * Configura o GPIO do led.
 **/
void init_led(void)
{
    gpio_config_t io_conf;                 
    io_conf.intr_type = GPIO_INTR_DISABLE;      ///< Disabilita interrupção.
    io_conf.mode = GPIO_MODE_OUTPUT;            ///< set as saídas mode.
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL; ///< Máscara de bit dos pinos que serão setadas.
    io_conf.pull_down_en = 0;                   ///< Desabilita o modo pull-down.                
    io_conf.pull_up_en = 0;                     ///< Desabilita o modo pull-up.       
    gpio_config(&io_conf);                      ///< Configura o GPIO.       
}

void sendData(uint8_t* data, int size) ///< SendData envia para a fila a informação que será transmitida para a serial.
{
    send_event_t event = { ///< Struct event envia dados mutuamente:
            .data = data,  ///< Vetor de informações.
            .size = size   ///< Tamanho das informações
    };

    /**
     * Tempo de espera para a função alocar o item na fila evitando o deadlock.
     **/

    xQueueSend(xQueue, (void*) &event, (TickType_t) 100);   
}

void sendLed(uint8_t data)
{
    xQueueSend(xQueueLed, (void*) &data, (TickType_t) 100); 
}
/**
 * Cálculo de checksum, a soma de todas as posições com excessão do próprio checksum.
 **/
uint8_t checksum(uint8_t* buff, size_t len)
{
    uint8_t sum;
    for (sum = 0 ; len != 0 ; len--)
        sum += *(buff++);
    return (uint8_t)sum;
}
/**
 * Task para o led
 **/
static void led_task(void *arg)
{
    uint8_t data = 0;      ///< Variavél que recebe os itens da fila.
    uint8_t led_state = 0; ///< Estado do led.

    gpio_set_level(GPIO_LED, led_state); 

    while (1) {
        if(xQueueReceive(xQueueLed, &data, ( TickType_t ) 100 ) == pdPASS)
        {
            if (data < 2) {
                gpio_set_level(GPIO_LED, data);
                led_state = data;
            } else {
                gpio_set_level(GPIO_LED, !led_state);
                led_state = !led_state;
            }
            
        }
    }
}
/**
 * Processo de transmissão, também apresenta tempo de espera na fila afim de evitar o deadlock. 
 * Porteriormente, irá passar a informação a ser enviada e o tamanho da informação.
 **/
static void tx_task(void *arg)
{
    send_event_t event = {0};

    while (1) {
        if(xQueueReceive(xQueue, &event, ( TickType_t ) 100 ) == pdPASS)
        {
            uart_write_bytes(UART_NUM_1, event.data, event.size);
        }
    }
}
/**
 * Aloca um buffer para receber informações, quando a informação é recebida ela é armazenada dentro do buffer data.
 * Quando o pacote é recebido a quantidade de bytes usado é informada.
 * Se o número de bytes for maior do que 0, inicia-se o protocolo
 **/
static void rx_task(void *arg)
{
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE+1);

    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 1000 / portTICK_RATE_MS);
        if (rxBytes > 0) {

            uint8_t start_byte = data[0]; ///< Inicio do byte.
            uint8_t cmd_byte = data[1];   ///< Byte de comando.
            uint8_t param_byte = data[2]; ///< Número de parâmentros.
            uint8_t checksum_byte = 0;    ///< Checksum.
            uint8_t* params;              ///< Vetor de parâmetro.

            int i = 0;                    

            uint8_t *resp = malloc(sizeof(uint8_t)); ///< Buffer de resposta padrão ACK.
            resp[0] = 6;


            if (start_byte == 1) {    ///< Se o start byte é igual a 1 o protocolo é iniciado.
                if (param_byte > 0) { ///< Se o parâmetro é maior que 0 então: 
            
                    params = (uint8_t*) malloc(sizeof(uint8_t) * param_byte); ///> Aloca um vetor de 1 byte vezes a quantidade de parâmetros, cada parâmetro ocupa 1 byte

                    for(i = 0; i < param_byte; i++) { ///< For para detecção de parâmetros
                        params[i] = data[3 + i];      
                    }

                    checksum_byte = data[3 + i];
                } else {
                    checksum_byte = data[3];
                }
                /**
                 * Faz a comparação entre o checksum byte e o checksum para verificar se o pacote foi recebido de forma integra (Sem perdas).
                 **/
                if (checksum_byte == checksum(data, sizeof(uint8_t) * (3 + i))) {
                    switch (cmd_byte) {
                    
                        case 0x00:///< comando desligar led
                            sendLed(0x00);
                            sendData(resp, sizeof(uint8_t));
                            break;
                            case 0x01:///< comando ligar led
                            sendLed(0x01);
                            sendData(resp, sizeof(uint8_t));
                            break;
                        case 0x02:///< comando toggle led
                            sendLed(0x02);
                            sendData(resp, sizeof(uint8_t));
                            break;
                        case 0x03:///< comando loopback
                            sendData(data, rxBytes);
                            break;
                    }
                }
           }
        }
    }

    free(data);
}

/**
 * Funções tasks criadas para rodarem simultâneamente.
 * Função principal do programa cria as filas para comunicação das funções tx task e led task, chama funções de inicialização para o GPIO do led
 * inicializar e configurar a UART, inicializao o NVS e inicializa o wifi.
 **/
void app_main(void)
{
    xQueue = xQueueCreate( 10, sizeof(send_event_t) );
    xQueueLed = xQueueCreate( 10, sizeof(uint8_t) );
    init_led();
    init_uart();
    init_nvs();
    wifi_init();

    xTaskCreate(rx_task, "uart_rx_task", 2048, NULL, 0, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 2048, NULL, 1, NULL);
    xTaskCreate(led_task, "led_task", 2048, NULL, 0, NULL);
}