#pragma once

#include "driver/uart.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the Simulink communication over UART.
 *
 * @param uart_num    UART port to use (typically UART_NUM_0).
 * @param baud_rate   Baud rate matching Simulink (e.g., 115200).
 * @param tx_vars     Number of variables (floats) to transmit to Simulink.
 * @param rx_vars     Number of variables (floats) to receive from Simulink.
 */
void simulink_comms_init(uart_port_t uart_num, uint32_t baud_rate, int tx_vars, int rx_vars);

/**
 * @brief Start the background tasks for receiving and transmitting.
 *
 * @param priority    Priority of the freeRTOS tasks (e.g., 5).
 * @param core_id     Core ID to pin tasks (0, 1, or tskNO_AFFINITY).
 * @param tx_rate_ms  The cycle time for transmission in milliseconds (e.g., 10).
 */
void simulink_comms_start_tasks(int priority, int core_id, int tx_rate_ms);

/**
 * @brief Thread-safe update of transmission variables.
 *        Other application tasks should call this to populate data bound for Simulink.
 *
 * @param tx_data     Pointer to array of floats mapping to your TX variables. 
 *                    Must match the length configured in `tx_vars`.
 */
void simulink_comms_set_tx_data(const float* tx_data);

/**
 * @brief Thread-safe retrieval of incoming variables.
 *        Other application tasks should call this to read data arriving from Simulink.
 *
 * @param rx_data     Pointer to array of floats where RX variables will be stored.
 *                    Must match the length configured in `rx_vars`.
 */
void simulink_comms_get_rx_data(float* rx_data);

#ifdef __cplusplus
}
#endif
