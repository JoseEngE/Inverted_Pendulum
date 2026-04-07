// main/bluetooth_telemetry.c
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "bluetooth_telemetry.h"
#include "pid_controller.h"
#include "pulse_counter.h"

#define SPP_SERVER_NAME "SPP_SERVER"
#define CONFIG_BT_DEVICE_NAME "Pendulo_Invertido"

static const char *TAG = "BT_TELEM";
static uint32_t spp_handle = 0;
static bool spp_connected = false;

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(TAG, "ESP_SPP_INIT_EVT");
        esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_SPP_CLOSE_EVT");
        spp_connected = false;
        break;
    case ESP_SPP_START_EVT:
        ESP_LOGI(TAG, "ESP_SPP_START_EVT");
        esp_bt_gap_set_device_name(CONFIG_BT_DEVICE_NAME);
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
        // No hacemos nada con los datos recibidos por ahora
        break;
    case ESP_SPP_CONG_EVT:
        break;
    case ESP_SPP_WRITE_EVT:
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_SPP_SRV_OPEN_EVT");
        spp_handle = param->srv_open.handle;
        spp_connected = true;
        break;
    case ESP_SPP_SRV_STOP_EVT:
        ESP_LOGI(TAG, "ESP_SPP_SRV_STOP_EVT");
        break;
    case ESP_SPP_UNINIT_EVT:
        ESP_LOGI(TAG, "ESP_SPP_UNINIT_EVT");
        break;
    default:
        break;
    }
}

static void bluetooth_telemetry_task(void *arg) {
    char packet[128];
    while (1) {
        if (spp_connected) {
            uint64_t time_ms = pid_get_run_time_ms();
            float pos_m = pid_get_car_position_m();
            float angle_rad = pulse_counter_get_angle_rad();

            // Formato CSV solicitado: tiempo_ms, posicion, angulo
            int len = snprintf(packet, sizeof(packet), "%llu,%.4f,%.4f\r\n", time_ms, pos_m, angle_rad);
            if (len > 0) {
                esp_spp_write(spp_handle, len, (uint8_t *)packet);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Enviar cada 50 ms (20 Hz)
    }
}

void bluetooth_telemetry_init(void) {
    esp_err_t ret;

    // Liberamos memoria de BLE que no usaremos si es posible
    ret = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "No se pudo liberar memoria BLE (quizas no este habilitado): %s", esp_err_to_name(ret));
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando controlador BT: %s", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(TAG, "Error habilitando BT clásico: %s", esp_err_to_name(ret));
        return;
    }

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando bluedroid: %s", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(TAG, "Error habilitando bluedroid: %s", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(NULL)) != ESP_OK) { // No necesitamos GAP callback por ahora
        ESP_LOGE(TAG, "Error registrando GAP cb: %s", esp_err_to_name(ret));
    }

    if ((ret = esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_VARIABLE, 0, NULL)) != ESP_OK) {
        ESP_LOGE(TAG, "Error seteando el pin GAP: %s", esp_err_to_name(ret));
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(TAG, "Error registrando SPP cb: %s", esp_err_to_name(ret));
        return;
    }

    esp_spp_cfg_t bt_spp_cfg = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = true,
        .tx_buffer_size = 0,
    };
    if ((ret = esp_spp_enhanced_init(&bt_spp_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando SPP: %s", esp_err_to_name(ret));
        return;
    }

    // Tarea con poco tamaño de stack necesario
    xTaskCreate(bluetooth_telemetry_task, "bt_telemetry_task", 4096, NULL, 3, NULL);

    ESP_LOGI(TAG, "Bluetooth Inicializado exitosamente.");
}
