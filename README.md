# Inverted Pendulum Project (ESP32 & ESP-IDF)

Este repositorio contiene el código fuente para el control de un **Péndulo Invertido** montado sobre un carro deslizante, desarrollado utilizando un microcontrolador ESP32 y el framework de desarrollo oficial de Espressif (ESP-IDF).

## 🚀 Arquitectura del Proyecto

El proyecto está diseñado bajo una arquitectura de sistema en tiempo real (RTOS) usando FreeRTOS. Se divide en varias tareas concurrentes que garantizan que el control (PID) tenga la máxima prioridad, mientras que las métricas y la interfaz de usuario se manejan en un segundo plano.

### Flujo de Tareas (FreeRTOS Tasks)
1. **`PID_Controller` (Prioridad 5):** Es el corazón del proyecto. Se ejecuta a una frecuencia fija (100 Hz / cada 10ms) y está encargado de leer la inclinación del péndulo, calcular el error respecto al punto de equilibrio (Setpoint 0) y generar un comando de velocidad y dirección usando un lazo de control PID continuo.
2. **`Motor_Control` (Prioridad 4):** Escucha permanentemente una cola de mensajes (`motor_command_queue`). Al recibir un comando del PID, inyecta la señal PWM directamente al driver del motor paso a paso con la nueva frecuencia (velocidad) exigida de forma ininterrumpida, permitiendo un movimiento suave y sin tirones.
3. **`button_handler_task` (Prioridad 4):** Se encarga de la interfaz física. Monitorea:
   - Botón de Activar/Desactivar PID.
   - Botones de movimiento manual (Jogging Izquierda / Derecha).
   - Sensores de Final de Carrera (Paradas de Emergencia).
   - Botón de cambio de vistas en la pantalla LCD.
   - Ejecución de la rutina automática de Calibración/Homing.
4. **`uart_echo_task` (Prioridad 3):** Permite recibir comandos y enviar telemetría a través del cable USB/Serial para graficar o depurar en el PC.
5. **`LCDDisplay` (Prioridad 3):** Tarea no crítica que actualiza una pantalla LCD 16x2 a través de I2C, mostrando vistas del estado del PID, posición, errores y menús de calibración.

---

## ⚙️ Componentes Principales e Implementación

* **Controlador PID (`pid_controller.c`):** 
  Utiliza una banda muerta (Dead Band) para evitar vibraciones minúsculas cuando el péndulo está virtualmente balanceado. También incluye un control en cascada leve para intentar mantener el carro (Odometría) cerca del centro geométrico del riel, sumando un pequeño "offset" al balance del péndulo para forzar al carro a retroceder lentamente.
* **Generador PWM (`pwm_generator.c`):** 
  Utiliza el periférico de hardware **LEDC** del ESP32. Envía pulsos asíncronos y continuos con un ciclo de trabajo del 50%. La velocidad se controla ajustando la frecuencia (`ledc_set_freq`) al vuelo sin bloquear la tarea, reaccionando instantáneamente a las exigencias del PID.
* **Contador de Pulsos (`pulse_counter.c`):** 
  Utiliza el hardware **PCNT** del ESP32 dedicado a leer el encoder incremental (cuadratura) acoplado al eje del péndulo rotativo. Ofrece lecturas precisas sub-milimétricas sin gastar recursos de la CPU.
* **Interfaz y Emergencias (`button_handler.c`):** 
  Garantiza la seguridad mecánica. Si alguno de los interruptores de límite de carrera es alcanzado, el PID se desactiva enviando y obligando a la frecuencia PWM ir a 0 Hz.

---

## 🛠️ Pines y Conexiones Físicas (Hardware Setup)

*(Basado en las definiciones del código)*

### Encoders (PCNT)
* **Pin de Fase A:** Definido internamente en `pulse_counter.h`
* **Pin de Fase B:** Definido internamente en `pulse_counter.h`

### Motor Driver (Señales Step / Dir)
* **Pin PWM (Pulsos de paso - STEP):** GPIO 32
* **Pin de Dirección (DIR):** GPIO 33

### Interfaz Física (Botones y Sensores)
* **Activar/Desactivar PID:** GPIO 18
* **Movimiento Manual Izquierda:** GPIO 16
* **Movimiento Manual Derecha:** GPIO 17
* **Cambiar Vista LCD:** GPIO 19
* **Botón de Calibración:** GPIO 15
* **Final de Carrera Izquierdo (Emergencia):** GPIO 34
* **Final de Carrera Derecho (Emergencia):** GPIO 35

---

## 🎯 Instrucciones de Uso y Puesta en Marcha

1. **Encendido:** Al iniciar, la pantalla LCD mostrará un mensaje de inicialización. El sistema arranca con el control PID **deshabilitado** por seguridad.
2. **Calibración (Homing):** Es el paso fundamental. Mantén el péndulo colgado (hacia abajo). Presiona el botón de **Calibración (GPIO 15)**.
   * El carro se moverá lentamente hasta tocar un final de carrera, luego se moverá al opuesto midiendo la distancia total del riel.
   * Tras chocar con ambos, el sistema calculará el centro exacto y el carro viajará a esa posición media.
   * El sistema asumirá automáticamente que esta posición de reposo del encoder mirando hacia el suelo es "-180°" y establecerá su punto de equilibrio (Setpoint) 180 grados en dirección opuesta (arriba puramente vertical).
3. **Movimiento Manual (Jogging):** Si el PID está apagado, puedes usar los botones manuales para desplazar el carro a conveniencia y despegarlo de las paredes.
4. **Activación del Péndulo:**
   * Levanta el péndulo manualmente hasta su punto aproximado de equilibrio superior (vertical).
   * Presiona el botón de **Activar PID (GPIO 18)**. 
   * El motor comenzará a rastrear la posición instantáneamente y mantendrá el péndulo balanceado. 
   * Volver a presionar el botón desactivará el motor y dejará caer el péndulo.
5. **Parada de Emergencia:** Si en cualquier momento de descontrol agresivo el marco del carro golpea uno de los límites izquierdo (34) o derecho (35), el motor se detendrá al instante protegiendo la estructura.

---

*Proyecto configurado y compilable exclusivamente bajo el entorno ESP-IDF Extension (No PlatformIO). Funciona sobre CMake Tools.*
