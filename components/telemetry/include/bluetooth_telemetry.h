// main/bluetooth_telemetry.h
#ifndef BLUETOOTH_TELEMETRY_H
#define BLUETOOTH_TELEMETRY_H

/**
 * @brief Inicializa el módulo de Bluetooth Classic SPP.
 * 
 * Esta función debe llamarse al inicio del sistema. Configurará 
 * el nombre del dispositivo, iniciará la pila de Bluetooth y 
 * se preparará para aceptar conexiones.
 */
void bluetooth_telemetry_init(void);

#endif // BLUETOOTH_TELEMETRY_H
