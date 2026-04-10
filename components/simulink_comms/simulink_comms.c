#include "simulink_comms.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "SIM_COMM";

static uart_port_t s_uart_num = UART_NUM_0;
static int s_tx_num = 0;
static int s_rx_num = 0;
static int s_tx_rate_ms = 10;

// Internal data buffers
static float *s_vars_tx = NULL;
static float *s_vars_rx = NULL;

// Port mutex for safe thread access
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

void simulink_comms_init(uart_port_t uart_num, uint32_t baud_rate, int tx_vars, int rx_vars) {
    s_uart_num = uart_num;
    s_tx_num = tx_vars;
    s_rx_num = rx_vars;

    // Allocate internal buffers
    if (s_tx_num > 0) s_vars_tx = (float*)calloc(s_tx_num, sizeof(float));
    if (s_rx_num > 0) s_vars_rx = (float*)calloc(s_rx_num, sizeof(float));

    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(s_uart_num, &uart_config);
    uart_set_pin(s_uart_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
                 
    // Install driver with standard buffer sizes 
    uart_driver_install(s_uart_num, 2048, 2048, 0, NULL, 0);
    ESP_LOGI(TAG, "Simulink comms initialized on UART%d. TX variables: %d, RX variables: %d", uart_num, tx_vars, rx_vars);
}

void simulink_comms_set_tx_data(const float* tx_data) {
    if (s_tx_num == 0 || s_vars_tx == NULL || tx_data == NULL) return;
    
    // Thread-safe copy
    portENTER_CRITICAL(&s_mux);
    memcpy(s_vars_tx, tx_data, s_tx_num * sizeof(float));
    portEXIT_CRITICAL(&s_mux);
}

void simulink_comms_get_rx_data(float* rx_data) {
    if (s_rx_num == 0 || s_vars_rx == NULL || rx_data == NULL) return;
    
    // Thread-safe copy
    portENTER_CRITICAL(&s_mux);
    memcpy(rx_data, s_vars_rx, s_rx_num * sizeof(float));
    portEXIT_CRITICAL(&s_mux);
}

static void tx_task(void *pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(s_tx_rate_ms);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    int bytes_per_transmission = sizeof(float) * s_tx_num;
    uint8_t *tx_buffer = (uint8_t*)malloc(2 + bytes_per_transmission); 
    
    if(!tx_buffer) {
        ESP_LOGE(TAG, "Failed to allocate TX buffer in task");
        vTaskDelete(NULL);
    }
    
    while (1) {
        tx_buffer[0] = 'V'; // Cabecera
        
        // Take a snapshot of the current TX data safely
        portENTER_CRITICAL(&s_mux);
        memcpy(&tx_buffer[1], s_vars_tx, bytes_per_transmission);
        portEXIT_CRITICAL(&s_mux);
        
        tx_buffer[1 + bytes_per_transmission] = '\n'; // Cola
        
        // Send data
        uart_write_bytes(s_uart_num, (const char *)tx_buffer, 2 + bytes_per_transmission);
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

static void rx_task(void *pvParameters) {
    const int rx_bytes_expected = sizeof(float) * s_rx_num;
    uint8_t *rx_buffer = (uint8_t*)malloc(rx_bytes_expected);
    
    if(!rx_buffer) {
        ESP_LOGE(TAG, "Failed to allocate RX buffer in task");
        vTaskDelete(NULL);
    }

    while (1) {
        int bytes_read = uart_read_bytes(s_uart_num, rx_buffer, rx_bytes_expected, pdMS_TO_TICKS(100));

        if (bytes_read == rx_bytes_expected) {
            // Write to global array safely
            portENTER_CRITICAL(&s_mux);
            memcpy(s_vars_rx, rx_buffer, rx_bytes_expected);
            portEXIT_CRITICAL(&s_mux);
            
        } else if (bytes_read > 0) {
            // Desaligned stream, clear buffer to realign
            uart_flush_input(s_uart_num);
        }
    }
}

void simulink_comms_start_tasks(int priority, int core_id, int tx_rate_ms) {
    s_tx_rate_ms = tx_rate_ms;
    
    if (s_tx_num > 0) {
        xTaskCreatePinnedToCore(tx_task, "SimTX", 4096, NULL, priority, NULL, core_id);
    }
    
    if (s_rx_num > 0) {
        xTaskCreatePinnedToCore(rx_task, "SimRX", 4096, NULL, priority, NULL, core_id);
    }
    
    ESP_LOGI(TAG, "Simulink comms tasks started (Core %d, Priority %d)", core_id, priority);
}
