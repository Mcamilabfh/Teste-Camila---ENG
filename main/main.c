#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"


static const int BUF_SIZE = 1024;

typedef struct {
    uint8_t* data;
    int size;
} send_event_t;

#define TXD_PIN (GPIO_NUM_1)
#define RXD_PIN (GPIO_NUM_3)
#define GPIO_LED (GPIO_NUM_2)
#define GPIO_OUTPUT_PIN_SEL (1ULL<<GPIO_LED) 

#define ESP_WIFI_SSID      "ENG TECNOLOGIA"
#define ESP_WIFI_PASS      "eng12345"
#define ESP_WIFI_CHANNEL   10
#define MAX_STA_CONN       100

QueueHandle_t xQueue;
QueueHandle_t xQueueLed;

static const char *TAG = "wifi ENG";

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
 * 
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

void init_nvs(void) {
    //Initialize Non Vollatile Storage
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void init_uart(void) {
    const uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, 2048, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void init_led(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

void sendData(uint8_t* data, int size)
{
    send_event_t event = {
            .data = data,
            .size = size
    };

    xQueueSend(xQueue, (void*) &event, (TickType_t) 100);
}

void sendLed(uint8_t data)
{
    xQueueSend(xQueueLed, (void*) &data, (TickType_t) 100);
}

uint8_t checksum(uint8_t* buff, size_t len)
{
    uint8_t sum;
    for (sum = 0 ; len != 0 ; len--)
        sum += *(buff++);
    return (uint8_t)sum;
}

static void led_task(void *arg)
{
    uint8_t data = 0;
    uint8_t led_state = 0;

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

static void rx_task(void *arg)
{
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE+1);

    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 1000 / portTICK_RATE_MS);
        if (rxBytes > 0) {

            uint8_t start_byte = data[0];
            uint8_t cmd_byte = data[1];
            uint8_t param_byte = data[2];
            uint8_t checksum_byte = 0;
            uint8_t* params;

            int i = 0;

            uint8_t *resp = malloc(sizeof(uint8_t));
            resp[0] = 6;

            if (start_byte == 1) {
                if (param_byte > 0) {
                    // cria um vetor de 1 byte vezes a quantidade de parâmetros, cada parâmetro ocupa 1 byte
                    params = (uint8_t*) malloc(sizeof(uint8_t) * param_byte);

                    for(i = 0; i < param_byte; i++) {
                        params[i] = data[3 + i];
                    }

                    checksum_byte = data[3 + i];
                } else {
                    checksum_byte = data[3];
                }

                if (checksum_byte == checksum(data, sizeof(uint8_t) * (3 + i))) {
                    switch (cmd_byte) {
                        case 0x00:
                            // comando ligar led
                            sendLed(0x01);
                            sendData(resp, sizeof(uint8_t));
                            break;
                        case 0x01:
                            // comando desligar led
                            sendLed(0x00);
                            sendData(resp, sizeof(uint8_t));
                            break;
                        case 0x02:
                            // comando toggle led
                            sendLed(0x02);
                            sendData(resp, sizeof(uint8_t));
                            break;
                        case 0x03:
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
 * 
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