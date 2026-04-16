# Inverted Pendulum — ESP32 & ESP-IDF

Control de un **péndulo invertido sobre carro deslizante** implementado en C con ESP-IDF (FreeRTOS), con soporte para dos estrategias de control intercambiables en tiempo real: **PID en cascada** y **realimentación de estados (LQI)**.

---

## 📁 Estructura del Proyecto

```
Inverted_Pendulum/
├── main/
│   └── main.c                   # Punto de entrada: inicialización y creación de tareas
├── components/
│   ├── controllers/             # Algoritmos de control
│   │   ├── pid_controller.c     # Controlador PID en cascada (ángulo + posición)
│   │   ├── state_space_controller.c  # Controlador LQI por realimentación de estados
│   │   └── include/
│   │       ├── pid_controller.h
│   │       └── state_space_controller.h
│   ├── hardware/                # Drivers de bajo nivel
│   │   ├── pwm_generator.c      # Generación PWM (LEDC) + tarea Motor_Control
│   │   ├── pulse_counter.c      # Lectura del encoder cuadratura (PCNT)
│   │   ├── button_handler.c     # Botones, jogging manual, finales de carrera, homing
│   │   └── include/
│   │       ├── pwm_generator.h  # Pines GPIO, motor_command_t, API pwm
│   │       ├── pulse_counter.h
│   │       └── button_handler.h
│   ├── telemetry/               # Canal de telemetría hacia el PC
│   │   ├── bluetooth_telemetry.c  # Telemetría en tiempo real vía Bluetooth Classic (SPP)
│   │   ├── uart_echo.c          # Echo UART para depuración liviana
│   │   └── include/
│   │       ├── bluetooth_telemetry.h
│   │       └── uart_echo.h
│   ├── simulink_comms/          # Puente binario ESP32 ↔ MATLAB/Simulink
│   │   ├── simulink_comms.c     # TX de 6 floats a 100 Hz por UART0
│   │   └── include/
│   │       └── simulink_comms.h
│   ├── display/                 # Interfaz visual LCD 16×2 (I2C)
│   │   ├── lcd_controller.c     # Tarea LCDDisplay, vistas y menú de calibración
│   │   ├── hd44780.c            # Driver bajo nivel HD44780
│   │   └── include/
│   ├── system_core/             # Estado global compartido entre componentes
│   │   ├── system_status.c      # Variables de estado: modo de control, vista LCD, jogging
│   │   └── include/
│   │       └── system_status.h
├── matlab/
│   └── simulacion_serial.slx   # Modelo Simulink para adquisición y análisis
├── sdkconfig                    # Configuración del ESP-IDF (generada por menuconfig)
├── sdkconfig.defaults           # Opciones por defecto fijadas del proyecto
└── CMakeLists.txt
```

---

## 🏗️ Arquitectura del Sistema

El sistema corre sobre **FreeRTOS** con múltiples tareas concurrentes ordenadas por prioridad. La tabla siguiente resume cada tarea activa:

| Tarea | Prioridad | Componente | Función |
|---|---|---|---|
| `StateSpace` | 5 | `controllers` | Controlador LQI activo cuando `USE_STATE_SPACE_CONTROLLER=1` |
| `PID_Controller` | 5 | `controllers` | Controlador PID en cascada (ángulo + posición) |
| `Motor_Control` | 4 | `hardware` | Escucha `motor_command_queue` y ejecuta PWM sin bloquear |
| `button_handler_task` | 4 | `hardware` | Botones, jogging, finales de carrera, homing |
| `SimTX` | 3 | `simulink_comms` | Telemetría binaria hacia Simulink a 100 Hz |
| `LCDDisplay` | 3 | `display` | Actualiza pantalla LCD (no crítico) |
| `BT Telemetry` | — | `telemetry` | Stream CSV por Bluetooth Classic (SPP) |

> **Selección de controlador:** La macro `#define USE_STATE_SPACE_CONTROLLER 1` en `main.c` activa el modo LQI. Ambas tareas se crean siempre, pero sólo una controla el motor a la vez mediante las funciones `ss_is_enabled()` / `pid_is_enabled()`.

---

## ⚙️ Componentes — Descripción Detallada

### `controllers` — Algoritmos de Control

#### PID en Cascada (`pid_controller.c`)
Implementa tres etapas encadenadas:

1. **Lazo de posición (lento):** Calcula un ángulo de *offset* para que el carro se desplace hacia el centro del riel.
2. **Lazo de ángulo (rápido, 100 Hz):** Suma el *offset* al setpoint vertical (0°) y genera una **aceleración** de corrección vía PID completo (P + I + D).
3. **Integrador de velocidad:** Convierte la aceleración en **velocidad de actuación** continua (m/s), que se traduce a frecuencia de pulsos para el driver.

Incluye **banda muerta** configurable alrededor de 0° y **anti-windup** en la rama integral.

API expuesta al módulo `simulink_comms`:
```c
float pid_get_car_position_m(void);
float pid_get_acceleration(void);
float pid_get_velocity(void);
float pid_get_angular_velocity(void);
uint64_t pid_get_run_time_ms(void);
```

#### Control por Realimentación de Estados — LQI (`state_space_controller.c`)
Controlador moderno basado en la dinámica linealizada del péndulo. Implementa un regulador LQI (LQR + integrador de posición) con las siguientes variables de estado:

| Variable | Descripción |
|---|---|
| `theta` | Ángulo del péndulo (rad) |
| `theta_dot_hat` | Velocidad angular estimada |
| `x_pos` | Posición del carro (m) |
| `x_dot` | Velocidad del carro (m/s) |
| `u_control` | Señal de control calculada |

API pública:
```c
bool  ss_is_enabled(void);
float ss_get_theta(void);
float ss_get_x_pos(void);
float ss_get_x_dot(void);
float ss_get_theta_dot_hat(void);
float ss_get_u_control(void);
```

---

### `hardware` — Drivers de Bajo Nivel

#### Generador PWM (`pwm_generator.c`)
- Usa el periférico **LEDC** del ESP32 (`LEDC_TIMER_0`, `LEDC_CHANNEL_0`).
- Pin STEP: **GPIO 32** — Pin DIR: **GPIO 33**.
- La `motor_control_task` recibe `motor_command_t {frequency, direction}` desde la cola `motor_command_queue` y ajusta la frecuencia **al vuelo** sin bloquear (`ledc_set_freq`), generando movimiento continuo asíncrono.
- Duty cycle fijo al 50% (resolución 1-bit para máxima frecuencia alcanzable).

#### Contador de Pulsos (`pulse_counter.c`)
- Usa el periférico **PCNT** del ESP32 para leer el encoder incremental en cuadratura acoplado al eje del péndulo.
- Expone la posición angular en radianes vía `pulse_counter_get_angle_rad()`.

#### Manejador de Botones y Emergencias (`button_handler.c`)
- **GPIO 18:** Activar/Desactivar controlador activo.
- **GPIO 16 / 17:** Jogging manual (izquierda / derecha).
- **GPIO 19:** Cambiar vista en LCD.
- **GPIO 15:** Iniciar rutina de calibración (Homing).
- **GPIO 34 / 35:** Finales de carrera — detienen el motor y deshabilitan el control inmediatamente.

---

### `simulink_comms` — Puente Binario ESP32 ↔ MATLAB/Simulink

Envía un paquete binario compacto a **100 Hz** por `UART_NUM_0` (USB/Serial):

```
[0x56 'V'] [float0] [float1] [float2] [float3] [float4] [float5] [0x0A '\n']
  1 byte  +       6 × 4 bytes (little-endian)                  +  1 byte  = 27 bytes
```

| Índice | Variable | Unidad |
|---|---|---|
| 0 | `time_ms` | ms |
| 1 | `theta` | rad |
| 2 | `x_pos` | m |
| 3 | `u_control` | m/s² (PID) / u (LQI) |
| 4 | `x_dot` | m/s |
| 5 | `theta_dot` | rad/s |

El formato es idéntico tanto para el modo PID como para el modo LQI — el selector interno es `ss_is_enabled()`. El modelo Simulink receptor está en `matlab/simulacion_serial.slx`.

Inicialización en `main.c`:
```c
simulink_comms_init(UART_NUM_0, 115200, 6, 0);
simulink_comms_start_tasks(3, tskNO_AFFINITY, 10); // 100 Hz
```

---

### `telemetry` — Canal Bluetooth

- **`bluetooth_telemetry.c`:** Emite telemetría en formato CSV por Bluetooth Classic (SPP). Permite monitoreo inalámbrico con herramientas como el script Python del proyecto o cualquier terminal serie BT.
- **`uart_echo.c`:** Canal UART auxiliar para depuración liviana (log de texto).

> ⚠️ El Bluetooth comparte recursos de radio con el scheduler del ESP32. Si se activa el control, asegurarse de usar `xTaskCreatePinnedToCore` para las tareas críticas de motor/controlador en el **Core 1**, relegando BT al Core 0.

---

### `display` — LCD 16×2 vía I2C

Muestra las siguientes vistas rotativas (botón GPIO 19):

| Vista | Contenido |
|---|---|
| `VIEW_MAIN_STATUS` | Estado del controlador activo + ángulo en grados |
| `VIEW_POSITION` | Posición del carro en cm |
| `VIEW_PID_GAINS` | Valores Kp, Ki, Kd actuales |
| `VIEW_VELOCITY` | Velocidad actual del carro |
| `VIEW_CONTROL_MODE` | Modo activo: PID / LQR |
| `VIEW_CALIBRATION` | Menú de homing (activado por GPIO 15) |

---

### `system_core` — Estado Global Compartido

Centraliza variables de estado accedidas desde múltiples tareas:

```c
// Modo de control activo
typedef enum { MODE_PID, MODE_STATE_SPACE } control_mode_t;

// Vista actual del LCD
typedef enum { VIEW_MAIN_STATUS, VIEW_POSITION, VIEW_PID_GAINS,
               VIEW_VELOCITY, VIEW_CONTROL_MODE, VIEW_CALIBRATION } lcd_view_state_t;

// Estado de movimiento manual
typedef enum { MANUAL_MOVE_NONE, MANUAL_MOVE_LEFT, MANUAL_MOVE_RIGHT } manual_move_state_t;
```

---

## 🛠️ Pines de Hardware Resumidos

| Señal | GPIO |
|---|---|
| STEP — pulsos al driver | 32 |
| DIR — dirección del motor | 33 |
| Activar/Desactivar control | 18 |
| Jogging izquierda | 16 |
| Jogging derecha | 17 |
| Cambiar vista LCD | 19 |
| Calibración / Homing | 15 |
| Final de carrera izquierdo | 34 |
| Final de carrera derecho | 35 |
| Encoder Fase A | Definido en `pulse_counter.h` |
| Encoder Fase B | Definido en `pulse_counter.h` |
| LCD I2C SDA / SCL | Definidos en `lcd_controller.h` |

---

## 🚀 Puesta en Marcha

### Requisitos
- **Toolchain:** ESP-IDF v5.x (extensión VSCode de Espressif recomendada).
- **Hardware:** ESP32 (no ESP32-S3), driver de motor paso a paso compatible con señales STEP/DIR, encoder incremental en cuadratura.
- **PC:** MATLAB/Simulink con el modelo `matlab/simulacion_serial.slx` para adquisición de datos.

### Compilar y Flashear
```bash
idf.py build
idf.py -p COMx flash monitor
```

### Secuencia de Arranque
1. **Encendido:** La LCD muestra "Bienvenidos… Iniciando". El control arranca **deshabilitado** por seguridad.
2. **Calibración (Homing):** Con el péndulo colgado hacia abajo, presionar **GPIO 15**.
   - El carro recorre el riel completo, toca ambos finales de carrera y se posiciona en el centro.
   - El encoder queda calibrado: posición colgante = −180°, posición vertical = 0° (setpoint).
3. **Jogging manual:** Con el control desactivado, usar **GPIO 16/17** para mover el carro libremente.
4. **Activación del control:**
   - Levantar el péndulo manualmente hasta la vertical.
   - Presionar **GPIO 18** para habilitar el controlador seleccionado (PID o LQI).
   - Volver a presionar apaga el control.
5. **Parada de emergencia:** Si el carro golpea un final de carrera (**GPIO 34/35**), el motor se detiene y el control se deshabilita instantáneamente.

---

## 📡 Adquisición de Datos con Simulink

1. Conectar el ESP32 al PC por USB.
2. Configurar el puerto COM correcto en el bloque Serial de `simulacion_serial.slx`.
3. Compilar el firmware con los logs de ESP_LOG **desactivados** para mantener el stream binario limpio (el módulo `simulink_comms` los suprime por defecto).
4. Iniciar la simulación. Los 6 canales se actualizarán a 100 Hz.

---

## 🗺️ Roadmap

### Deuda Técnica
- [ ] **Migrar driver PCNT** al API de ESP-IDF v5 (`driver/pulse_cnt.h`) — thread-safe y con callbacks de índice en hardware.
- [ ] **Reemplazar `vTaskDelayUntil`** en el PID por un callback de `esp_timer` para eliminar jitter del RTOS.
- [ ] **Calcular `dt` real** con `esp_timer_get_time()` en cada iteración del PID en lugar de asumir 10 ms fijos.
- [ ] **`xTaskCreatePinnedToCore`:** Anclar tareas de control/motor al Core 1 y BT/LCD al Core 0.

### Mejoras Funcionales
- [ ] **Re-homing dinámico:** Cuando el carro toca un final de carrera, sobrescribir `g_car_position_pulses` para resincronizar la odometría.
- [ ] **Watchdog de caída:** Deshabilitar el PID automáticamente si `|theta| > 45°` para evitar colisiones violentas.
- [ ] **NVS:** Guardar ganancias Kp, Ki, Kd sintonizadas en la Flash para que sobrevivan reinicios.
- [ ] **`config.h` centralizado:** Unificar pines, dimensiones mecánicas y constantes dispersas en un único cabecero maestro.
- [ ] **Observador de Luenberger:** Estimar estados no medibles directamente para mejorar la robustez del controlador LQI.

### Testing
Se recomienda el framework **Unity** (integrado en ESP-IDF) para pruebas unitarias de `PID_Compute()`:
- Lazo proporcional puro (salida ∝ Kp × error).
- Anti-windup: error sostenido grande no desborda la integral.
- Banda muerta: errores menores al umbral producen salida 0.
- Derivada: señal en diente de sierra genera signos correctos sin `NaN`.

---

*Proyecto compilable exclusivamente con **ESP-IDF** (CMake + Ninja). No compatible con PlatformIO.*
