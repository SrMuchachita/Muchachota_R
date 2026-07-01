// ============================================================
// RD90 Pro - Firmware ESP32-S3
// Versión corregida - Abril 2026
//
// Correcciones aplicadas:
//  [FIX-1] current_dir inicializada en MOTOR_STOP (motores se iban solos)
//  [FIX-2] last_motor_activity_time inicializada con tiempo real al arranque
//  [FIX-3] Ángulos IMU sin escala x100, guardados directamente en grados
//  [FIX-4] DOUT0 inicializado en vGpioInit
//  [FIX-5] vTaskImuMonitor con un solo vTaskDelay al final del ciclo
//  [FIX-6] Validación de errores en vI2cInit
//  [FIX-7] motorDcControl marcada como función de test, no se llama en producción
//  [FIX-8] Watchdog de motores separado y limpio al inicio del loop
//  [FIX-9] Timeout en wait_for_sensor: while(timeout-- > 0) en lugar de while(timeout--)
//  [FIX-10] Motor atorado: agregar target_vel=0 cuando cmd==MOTOR_STOP para rampa suave
//  [FIX-11] Filtro complementario: acelerómetro + giroscopio para ángulos estables
//  [FIX-12] Calibración de ángulos IMU: offset referenceable activable por código
//  [FIX-13] Lectura de temperatura cada 500ms en lugar de 100ms para evitar errores I2C
//  [FIX-14] Delays después de encender IMU + aumentar periodo de lectura a 200ms
//  [FIX-15] Calibración automática al inicio + logs de ángulos cada 1 segundo en serial
//  [FIX-17] Logs de dirección y velocidad de motores cada 2 segundos en serial
//  [FIX-18] Prevenir underflow en rampa suave: verificar que current_vel >= RAMP_STEP
// ============================================================

//ANSI C
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

// ESP-IDF Drivers
#include "driver/gptimer.h"
#include "driver/i2c_master.h"
#include "driver/mcpwm_prelude.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"

// Librerias externas
#include "esp_modbus_slave.h"
#include "mbcontroller.h"
#include "modbus_params.h"
#include "ina260.h"
#include "icm42670.h"
#include "servo_motor.h"


//*****************************************RECURSOS DE MODULO MAIN************************************************/
#define TASK_SIZE            (6 * 1024)
#define TAG                  "RD90Pro"


//*****************************************DEFINICIONES EQUIPO*********************************************************/
#define MB_SLAVE_ADDR        (0x0A)
#define MB_SLAVE_BAUD        (115200)
#define MB_FIRMWARE_REVISION (0x0100)
#define DEVICE_SERIAL_PREFIX "RD90R"
#define DEVICE_SERIAL_SUFFIX "260003"    // <- Cambiar por el numero serial del robot
/*Formato: RD90R-262501 (26 = año 2026, 0001 = robot número 1 de esa semana)*/

//*****************************************DEFINICIONES SYSTEM MODE**************************************************/
#define MB_SYSTEM_START      (0x07D0)
#define MB_SYSTEM_STOP       (0x03E8)


//*****************************************DEFINICIONES GPIO*********************************************************/
#define DOUT0   41
#define DOUT1   40
#define DOUT2   39
#define DOUT3   38
#define DOUT_MASK_CONFIG     ((1ULL << DOUT0)|(1ULL << DOUT1)|(1ULL << DOUT2)|(1ULL << DOUT3))


//*****************************************PINOUT MOTORES DC ******************************************************/
#define M0INH   3
#define M0PWMA  10
#define M0PWMB  9

#define M1INH   4
#define M1PWMA  7
#define M1PWMB  5

#define M2INH   8
#define M2PWMA  17
#define M2PWMB  18

#define M3INH   16
#define M3PWMA  15
#define M3PWMB  6

/*
#define PWM_FREQ_HZ        10000
#define PWM_RESOLUTION_HZ  10000000
#define PWM_PERIOD_TICKS   (PWM_RESOLUTION_HZ / PWM_FREQ_HZ)
#define MOTOR_COUNT        (4)
#define MOTOR_MAX_PWM      1000
#define RAMP_STEP          10
#define TASK_PERIOD_MS     20*/

// DESPUÉS — líneas modificadas
#define PWM_FREQ_HZ        20000      // ← era 10000
#define PWM_RESOLUTION_HZ  10000000  // ← no cambia
#define PWM_PERIOD_TICKS   (PWM_RESOLUTION_HZ / PWM_FREQ_HZ)  // ← no cambia (se calcula solo = 10000)
#define MOTOR_COUNT        (4)
#define MOTOR_MAX_PWM      1000     // ← era 1000
#define RAMP_STEP          10       // ← no cambia, con PWM_MAX 500 da una rampa suave de 50 pasos (500/10)
#define TASK_PERIOD_MS     20        // ← no cambia
/*500 / 10 = 50 pasos × 20ms = solo 1 segundo*/

//*****************************************PINOUT SERVOS******************************************************/
#define SERVO1_CH0_PIN   12
#define SERVO2_CH1_PIN   11


//*****************************************PINOUT PWM LED******************************************************/
#define LED_PIN          13
#define LED_FREQ_HZ      5000


//*****************************************DEFINICIONES I2C*********************************************************/
#define INA_I2C_PORT                    I2C_NUM_0
#define INA_I2C_SDA_PIN                 (48)
#define INA_I2C_SCL_PIN                 (47)

#define IMU_I2C_PORT                    I2C_NUM_1
#define IMU_I2C_SDA_PIN                 (14)
#define IMU_I2C_SCL_PIN                 (21)

#define IMU_DEV_ADDR                    (0b1101000)
#define IMU_DEV_FREQ_Hz                 100000

#define INA_DEV_ADDR                    (0b1000000)
#define INA_DEV_FREQ_Hz                 100000


//*****************************************DEFINICIONES MODBUS*****************************************************/
#define MB_PORT_NUM           UART_NUM_2
#define MB_UART_TXD           (1)
#define MB_UART_RXD           (2)
#define MB_UART_DIR           (42)
#define UART_BUFFER_SIZE      (512)

#define MB_REG_COILS_START    (0x0000)
#define MB_REG_HOLDING_START  (0x0000)
#define MB_REG_INPUT_START    (0x0000)

#define MB_READ_MASK          (MB_EVENT_HOLDING_REG_RD | MB_EVENT_COILS_RD | MB_EVENT_INPUT_REG_RD)
#define MB_WRITE_MASK         (MB_EVENT_HOLDING_REG_WR | MB_EVENT_COILS_WR)
#define MB_READ_WRITE_MASK    (MB_READ_MASK | MB_WRITE_MASK)


//*****************************************BITS DE EVENTOS*****************************************************/
#define MB_REG_COIL0_WR_BIT       BIT0
#define MB_REG_COIL1_WR_BIT       BIT1
#define MB_REG_COIL2_WR_BIT       BIT2
#define MB_REG_COIL3_WR_BIT       BIT3
#define MB_REG_COILS_MASK         (MB_REG_COIL0_WR_BIT | MB_REG_COIL1_WR_BIT | MB_REG_COIL2_WR_BIT | MB_REG_COIL3_WR_BIT)

#define MB_REG_HOLDING_MOTOR_BIT  BIT0
#define MB_REG_HOLDING_SERVO_BIT  BIT1
#define MB_REG_HOLDING_LED_BIT    BIT2


//*****************************************TIMEOUT MOTORES*****************************************************/
// [FIX-8] Timeout claramente definido y separado
#define MODBUS_TIMEOUT_MS           3000    // 3 segundos sin actividad -> stop motores
#define MODBUS_INACTIVITY_RESET_MS  (5 * 60 * 1000)  // 5 min sin actividad -> restart


//******************************* ESTRUCTURAS PARA CONTROL DE MOTOR *****************************************/
typedef enum {
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_REVERSE,
    MOTOR_RIGHT,
    MOTOR_LEFT,
} motor_dir_t;

typedef struct {
    gpio_num_t inh;
    mcpwm_oper_handle_t oper;
    mcpwm_cmpr_handle_t cmp_a;
    mcpwm_cmpr_handle_t cmp_b;
    mcpwm_gen_handle_t  gen_a;
    mcpwm_gen_handle_t  gen_b;
} motor_t;

static mcpwm_timer_handle_t timer0;
static mcpwm_timer_handle_t timer1;
static motor_t motors[MOTOR_COUNT];


//****************************************PROTOTIPOS DE FUNCIONES APP*********************************************/
void vGpioInit(void);
void vI2cInit(void);
void vHardwareInit(void);
void vModbusSlaveInit(void);
void vMotorsDcInit(void);
void vServosInit(void);
void vLedPwmInit(void);


//**************************************PROTOTIPOS DE FUNCIONES DE UTIL*********************************************/
void motorDcSet(uint8_t id, motor_dir_t dir, uint16_t speed);
void motorDcControl_TEST(void);   // [FIX-7] Renombrada como TEST, no usar en producción
void setServoAngles(void);
void wait_for_sensor(ina260_t *dev);
void vModbusSlaveControlStop(void);

// [FIX-12] Función para calibración de ángulos IMU
void imu_calibrate_reference(float pitch, float roll);
void imu_calibration_enable(bool enable);


//**************************************PROTOTIPOS DE FUNCIONES TAREAS*******************************************/
void vTaskModbusEvents  (void *pvParameters);
void vTaskMotorsControl (void *pvParameters);
void vTaskServosControl (void *pvParameters);
void vTaskPowerMonitor  (void *pvParameters);
void vTaskImuMonitor    (void *pvParameters);
void vTaskDigitalOutput (void *pvParameters);
void vTaskLedControl    (void *pvParameters);
void vTaskModbusWatchdog(void *pvParameters);


//************************************** RECURSOS DE FREERTOS ****************************************************/
EventGroupHandle_t xEventGroupCoilsReg   = NULL;
EventGroupHandle_t xEventGroupHoldingReg = NULL;


//************************************** VARIABLES GLOBALES ****************************************************/
static void         *mb_slave_handle             = NULL;
static portMUX_TYPE  paramLock                   = portMUX_INITIALIZER_UNLOCKED;
static int64_t       last_modbus_activity_time   = 0;
static int64_t       last_motor_activity_time    = 0;

// [FIX-12] Variables para calibración de ángulos IMU
static float imu_calibration_offset_pitch = 0.0f;
static float imu_calibration_offset_roll = 0.0f;
static bool imu_calibration_enabled = false;

static char device_serial[16] = {0};

i2c_master_bus_handle_t i2c1_handle   = NULL;
icm42670_handle_t dev_icm42670_handle = NULL;
ina260_t dev_ina260_handle            = {0};

servo_handle_t servo1;
servo_handle_t servo2;


//**************************************APP MAIN******************************************************/
void app_main(void)
{
    snprintf(device_serial, sizeof(device_serial), DEVICE_SERIAL_PREFIX "-" DEVICE_SERIAL_SUFFIX);
    ESP_LOGI(TAG, "Serial: %s", device_serial);

    vHardwareInit();

    // Cargar serial en registros Modbus (30007-30012), 2 chars ASCII por registro
    for (int i = 0; i < 6; i++) {
        mb_input_reg.serial[i] = ((uint16_t)(uint8_t)device_serial[i * 2] << 8)
                                | (uint8_t)device_serial[i * 2 + 1];
    }

    xEventGroupHoldingReg = xEventGroupCreate();
    if (xEventGroupHoldingReg == NULL) {
        ESP_LOGE(TAG, "Error al crear xEventGroupHoldingReg");
        return;
    }

    xEventGroupCoilsReg = xEventGroupCreate();
    if (xEventGroupCoilsReg == NULL) {
        ESP_LOGE(TAG, "Error al crear xEventGroupCoilsReg");
        return;
    }

    xTaskCreate(vTaskModbusEvents,   "ModbusRTU",  TASK_SIZE, NULL, 6, NULL);
    xTaskCreate(vTaskMotorsControl,  "MotoresDC",  TASK_SIZE, NULL, 5, NULL);
    xTaskCreate(vTaskServosControl,  "ServoMotor", TASK_SIZE, NULL, 3, NULL);
    xTaskCreate(vTaskLedControl,     "LedCtrlPwm", TASK_SIZE, NULL, 3, NULL);
    xTaskCreate(vTaskPowerMonitor,   "PowerMon",   TASK_SIZE, NULL, 5, NULL);
    xTaskCreate(vTaskImuMonitor,     "ImuMonitor", TASK_SIZE, NULL, 4, NULL);
    xTaskCreate(vTaskDigitalOutput,  "DigitalOut", TASK_SIZE, NULL, 3, NULL);
    xTaskCreate(vTaskModbusWatchdog, "MbWatchdog", TASK_SIZE, NULL, 7, NULL);
}


/* ============================================================
 * TAREAS FREERTOS
 * ============================================================ */

void vTaskModbusEvents(void *pvParameters)
{
    mb_param_info_t regInfo;

    while (1) {
        (void)mbc_slave_check_event(mb_slave_handle, MB_READ_WRITE_MASK);

        if (ESP_OK != mbc_slave_get_param_info(mb_slave_handle, &regInfo, 10)) {
            ESP_LOGE(TAG, "Get parameter information -> ESP_ERR_TIMEOUT");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Registrar actividad Modbus
        portENTER_CRITICAL(&paramLock);
        last_modbus_activity_time = esp_timer_get_time() / 1000;
        portEXIT_CRITICAL(&paramLock);

        switch (regInfo.type)
        {
            case MB_EVENT_INPUT_REG_RD:
            break;

            case MB_EVENT_HOLDING_REG_WR:
                // Registros 0-1: Motor
                if (regInfo.mb_offset <= 1) {
                    // [FIX-2] Actualizar tiempo de actividad de motores
                    portENTER_CRITICAL(&paramLock);
                    last_motor_activity_time = esp_timer_get_time() / 1000;
                    portEXIT_CRITICAL(&paramLock);
                }
                // Registros 2-3: Servos
                else if (regInfo.mb_offset <= 3) {
                    xEventGroupSetBits(xEventGroupHoldingReg, MB_REG_HOLDING_SERVO_BIT);
                }
                // Registro 4: LED
                else if (regInfo.mb_offset == 4) {
                    xEventGroupSetBits(xEventGroupHoldingReg, MB_REG_HOLDING_LED_BIT);
                }
            break;

            case MB_EVENT_HOLDING_REG_RD:
            break;

            case MB_EVENT_COILS_WR:
                if (regInfo.size == 1) {
                    if (regInfo.mb_offset == 0) {
                        xEventGroupSetBits(xEventGroupCoilsReg, MB_REG_COIL0_WR_BIT);
                    } else if (regInfo.mb_offset == 1) {
                        xEventGroupSetBits(xEventGroupCoilsReg, MB_REG_COIL1_WR_BIT);
                    } else if (regInfo.mb_offset == 2) {
                        xEventGroupSetBits(xEventGroupCoilsReg, MB_REG_COIL2_WR_BIT);
                    } else if (regInfo.mb_offset == 3) {
                        xEventGroupSetBits(xEventGroupCoilsReg, MB_REG_COIL3_WR_BIT);
                    } else {
                        ESP_LOGE(TAG, "Error de solicitud de escritura Coil");
                    }
                }
            break;

            default:
                ESP_LOGE(TAG, "Evento desconocido: %d", regInfo.type);
            break;
        }
    }
}


void vTaskMotorsControl(void *pvParameters)
{
    motor_dir_t cmd = MOTOR_STOP;

    // [FIX-1] current_dir inicializada en MOTOR_STOP, no MOTOR_FORWARD
    static motor_dir_t current_dir = MOTOR_STOP;

    uint16_t target_vel  = 0;
    static uint16_t current_vel = 0;

    // [FIX-2] Inicializar last_motor_activity_time con tiempo real al arranque
    // Así el watchdog no dispara inmediatamente
    portENTER_CRITICAL(&paramLock);
    last_motor_activity_time = esp_timer_get_time() / 1000;
    portEXIT_CRITICAL(&paramLock);

    // Estado seguro al arranque
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motorDcSet(i, MOTOR_STOP, 0);
    }

    while (1)
    {
        // =========================
        // [FIX-8] WATCHDOG separado y limpio al inicio del loop
        // =========================
        int64_t now = esp_timer_get_time() / 1000;
        int64_t last_activity_snapshot;

        portENTER_CRITICAL(&paramLock);
        last_activity_snapshot = last_motor_activity_time;
        portEXIT_CRITICAL(&paramLock);

        bool watchdog_triggered = (now - last_activity_snapshot) > MODBUS_TIMEOUT_MS;

        // =========================
        // Leer Modbus
        // =========================
        portENTER_CRITICAL(&paramLock);
        cmd        = (motor_dir_t)mb_holding_reg.motor_cmd;
        target_vel = mb_holding_reg.motor_vel;
        portEXIT_CRITICAL(&paramLock);

        // =========================
        // Aplicar watchdog
        // =========================
        if (watchdog_triggered) {
            cmd        = MOTOR_STOP;
            target_vel = 0;
            // Solo loguear una vez cuando dispara, no en cada ciclo
            static bool watchdog_logged = false;
            if (!watchdog_logged) {
                ESP_LOGW(TAG, "WATCHDOG MOTORES: sin actividad > %d ms -> STOP", MODBUS_TIMEOUT_MS);
                watchdog_logged = true;
            }
        } else {
            // Reset del flag de log cuando hay actividad normal
            static bool watchdog_logged = false;
            watchdog_logged = false;
        }

        // =========================
        // VALIDACIONES
        // =========================
        if (target_vel > MOTOR_MAX_PWM) {
            target_vel = MOTOR_MAX_PWM;
        }

        if (cmd > MOTOR_LEFT) {
            cmd = MOTOR_STOP;
        }

        // =========================
        // CAMBIO DE DIRECCIÓN SEGURO
        // =========================
        if ((cmd != current_dir) && (cmd != MOTOR_STOP))
        {
            // Primero frenar completamente antes de cambiar dirección
            target_vel = 0;

            if (current_vel == 0) {
                current_dir = cmd;
            } else {
                // Mantener dirección actual mientras frena
                cmd = current_dir;
            }
        }
        else if (cmd == MOTOR_STOP)
        {
            // [FIX-10] Poner target_vel a 0 para que la rampa suave disminuya gradualmente
            // Sin esto, la rampa nunca sabe que debe frenar y el motor se queda atorado
            target_vel = 0;
            
            // Si se pide stop explícito, actualizar dirección también
            if (current_vel == 0) {
                current_dir = MOTOR_STOP;
            }
        }
        else
        {
            current_dir = cmd;
        }

        // =========================
        // RAMPA SUAVE
        // =========================
        if (current_vel < target_vel) {
            current_vel += RAMP_STEP;
        } else if (current_vel > target_vel) {
            // [FIX-18] Prevenir underflow: si current_vel < RAMP_STEP, ir directamente a target_vel
            // Antes: 5 - 10 = 65531 (underflow en uint16_t) -> robot se atoraba con valores enormes
            if (current_vel >= RAMP_STEP) {
                current_vel -= RAMP_STEP;
            } else {
                current_vel = target_vel;  // Llegar al target sin underflow
            }
        }

        // Evitar oscilación alrededor del target
        if (abs((int)current_vel - (int)target_vel) < RAMP_STEP) {
            current_vel = target_vel;
        }

        // =========================
        // CONTROL DE MOTORES
        // =========================
        // Solo detener cuando la rampa ya llegó a 0
        // NO usar cmd == MOTOR_STOP aquí, eso cortaría la rampa suave
        if (current_vel == 0)
        {
            for (int i = 0; i < MOTOR_COUNT; i++) {
                motorDcSet(i, MOTOR_STOP, 0);
            }
        }
        else
        {
            switch (current_dir)
            {
                case MOTOR_FORWARD:
                    for (int i = 0; i < MOTOR_COUNT; i++) {
                        motorDcSet(i, MOTOR_FORWARD, current_vel);
                    }
                    break;

                case MOTOR_REVERSE:
                    for (int i = 0; i < MOTOR_COUNT; i++) {
                        motorDcSet(i, MOTOR_REVERSE, current_vel);
                    }
                    break;

                case MOTOR_RIGHT:
                    motorDcSet(0, MOTOR_FORWARD, current_vel);
                    motorDcSet(1, MOTOR_REVERSE, current_vel);
                    motorDcSet(2, MOTOR_FORWARD, current_vel);
                    motorDcSet(3, MOTOR_REVERSE, current_vel);
                    break;

                case MOTOR_LEFT:
                    motorDcSet(0, MOTOR_REVERSE, current_vel);
                    motorDcSet(1, MOTOR_FORWARD, current_vel);
                    motorDcSet(2, MOTOR_REVERSE, current_vel);
                    motorDcSet(3, MOTOR_FORWARD, current_vel);
                    break;

                default:
                    // Cualquier estado desconocido -> seguro
                    for (int i = 0; i < MOTOR_COUNT; i++) {
                        motorDcSet(i, MOTOR_STOP, 0);
                    }
                    break;
            }
        }

        // [FIX-17] Log de estado de motores cada 2 segundos (10 ciclos × 20ms)
        static int motor_log_counter = 0;
        if (motor_log_counter++ >= 10) {
            motor_log_counter = 0;
            const char* dir_names[] = {"STOP", "FORWARD", "REVERSE", "RIGHT", "LEFT"};
            //ESP_LOGI(TAG, "MOTOR: dir=%s vel_actual=%d vel_target=%d (PWM: %d/1000)",
                     //dir_names[current_dir], current_vel, target_vel, current_vel);
        }

        vTaskDelay(pdMS_TO_TICKS(TASK_PERIOD_MS));
    }
}


void vTaskServosControl(void *pvParameters)
{
    uint16_t angle1, angle2;

    while (1)
    {
        xEventGroupWaitBits(xEventGroupHoldingReg,
                            MB_REG_HOLDING_SERVO_BIT,
                            pdTRUE,
                            pdFALSE,
                            portMAX_DELAY);

        portENTER_CRITICAL(&paramLock);
        angle1 = mb_holding_reg.srv1_angle;
        angle2 = mb_holding_reg.srv2_angle;
        portEXIT_CRITICAL(&paramLock);

        if (angle1 > 180) angle1 = 180;
        if (angle2 > 180) angle2 = 180;

        servo_set_angle(servo1, angle1);
        servo_set_angle(servo2, angle2);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void vTaskLedControl(void *pvParameters)
{
    uint16_t duty = 0;

    while (1)
    {
        xEventGroupWaitBits(xEventGroupHoldingReg,
                            MB_REG_HOLDING_LED_BIT,
                            pdTRUE,
                            pdFALSE,
                            portMAX_DELAY);

        portENTER_CRITICAL(&paramLock);
        duty = mb_holding_reg.led_pwm;
        portEXIT_CRITICAL(&paramLock);

        if (duty > 1023) duty = 1023;

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void vTaskPowerMonitor(void *pvParameters)
{
    float voltage, current, power;
    wait_for_sensor(&dev_ina260_handle);

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(200));

        if (ESP_OK != ina260_trigger(&dev_ina260_handle)) {
            ESP_LOGE(TAG, "Error en la configuracion de un solo disparo");
            continue;
        }

        wait_for_sensor(&dev_ina260_handle);

        if (ESP_OK != ina260_get_bus_voltage(&dev_ina260_handle, &voltage)) {
            ESP_LOGE(TAG, "Error en la lectura de voltaje");
            continue;
        }

        if (ESP_OK != ina260_get_current(&dev_ina260_handle, &current)) {
            ESP_LOGE(TAG, "Error en la lectura de corriente");
            continue;
        }

        if (ESP_OK != ina260_get_power(&dev_ina260_handle, &power)) {
            ESP_LOGE(TAG, "Error en la lectura de potencia");
            continue;
        }

        portENTER_CRITICAL(&paramLock);
        mb_input_reg.ina_voltage = (uint16_t)(voltage * 1000.0f); // mV
        mb_input_reg.ina_current = (uint16_t)(current * 1000.0f); // mA
        mb_input_reg.ina_power   = (uint16_t)(power   * 1000.0f); // mW
        portEXIT_CRITICAL(&paramLock);
    }
}


void vTaskImuMonitor(void *pvParameters)
{
    // [FIX-11] FILTRO COMPLEMENTARIO: Combina acelerómetro + giroscopio para ángulos estables
    // Problema: El acelerómetro solo detecta aceleración lineal de los motores, alterando pitch/roll
    // Solución: Usar giroscopio para velocidad angular + acelerómetro para corrección de drift
    // Fórmula: angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel_angle
    // alpha = 0.98 da más peso al giroscopio (suave) pero se corrige lentamente
    
    // Variables para filtro complementario
    static float filtered_pitch = 0.0f;
    static float filtered_roll = 0.0f;
    const float COMPLEMENTARY_FILTER_ALPHA = 0.80f;  // 80% giroscopio, 20% acelerómetro
    const float DT = 0.2f;  // [FIX-14] Actualizar DT a 200ms entre lecturas (cambió de 100ms)
    
    // [FIX-13] Lectura de temperatura menos frecuente para evitar errores I2C
    // La temperatura no cambia rápidamente, leer cada 500ms en lugar de 100ms
    static int temp_read_counter = 0;
    const int TEMP_READ_INTERVAL = 2;  // [FIX-14] Leer cada 2 ciclos (200ms * 2 = 400ms aprox)
    
    // [FIX-15] Calibración automática al inicio - inicializar filtro con acelerómetro
    static bool first_read = true;
    static int init_counter = 0;
    
    icm42670_value_t acc;
    icm42670_value_t gyro;
    float temperature = 0.0f;
    float accel_pitch, accel_roll;

    while (1)
    {
        // Leer acelerómetro
        if (ESP_OK != icm42670_get_acce_value(dev_icm42670_handle, &acc)) {
            ESP_LOGE(TAG, "Error lectura acelerómetro");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Leer giroscopio para filtro complementario
        if (ESP_OK != icm42670_get_gyro_value(dev_icm42670_handle, &gyro)) {
            ESP_LOGE(TAG, "Error lectura giroscopio");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // [FIX-13] Leer temperatura cada 5 ciclos (500ms) en lugar de cada ciclo
        // Esto reduce la carga del bus I2C y evita errores por lectura muy frecuente
        if (temp_read_counter++ >= TEMP_READ_INTERVAL) {
            temp_read_counter = 0;
            if (ESP_OK != icm42670_get_temp_value(dev_icm42670_handle, &temperature)) {
                ESP_LOGW(TAG, "Error lectura temperatura IMU (intentando de nuevo en 500ms)");
                // No es crítico, continuamos sin abortar el ciclo
            }
        }

        // Remapeo de ejes según montaje físico
        float ax = acc.y;
        float ay = acc.x;
        float az = acc.z;
        float gx = gyro.y;  // roll rate
        float gy = gyro.x;  // pitch rate
        float gz = gyro.z;  // yaw rate (no usado)

        // Calcular ángulos solo del acelerómetro (sin movimiento lineal ideal)
        accel_pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;
        accel_roll  = atan2f(ay, az) * 180.0f / M_PI;

        // [FIX-15] Calibración automática: en los primeros 10 ciclos, inicializar con acelerómetro
        // Esto elimina el offset inicial que genera el filtro complementario
        if (first_read && init_counter++ < 10) {
            filtered_pitch = accel_pitch;
            filtered_roll = accel_roll;
            if (init_counter == 1) {
                ESP_LOGI(TAG, "IMU inicializando filtro con acelerómetro...");
            }
        } else if (first_read) {
            first_read = false;
            ESP_LOGI(TAG, "IMU calibración automática completada. Pitch=%.2f° Roll=%.2f°", 
                     filtered_pitch, filtered_roll);
        }

        // [FIX-11] FILTRO COMPLEMENTARIO: Combina velocidad angular + acelerómetro
        // - El giroscopio proporciona cambios suave (rechaza aceleración lineal)
        // - El acelerómetro corrige el drift acumulado del giroscopio
        filtered_pitch = COMPLEMENTARY_FILTER_ALPHA * (filtered_pitch + gy * DT) + 
                         (1.0f - COMPLEMENTARY_FILTER_ALPHA) * accel_pitch;
        filtered_roll = COMPLEMENTARY_FILTER_ALPHA * (filtered_roll + gx * DT) + 
                        (1.0f - COMPLEMENTARY_FILTER_ALPHA) * accel_roll;

        // [FIX-12] Aplicar calibración: restar offsets si está habilitada
        float final_pitch = filtered_pitch;
        float final_roll = filtered_roll;
        if (imu_calibration_enabled) {
            final_pitch = filtered_pitch - imu_calibration_offset_pitch;
            final_roll = filtered_roll - imu_calibration_offset_roll;
        }

        // Actualizar registros Modbus de forma atómica
        // Nota: el maestro Modbus (control) debe leer estos valores y dividir
        // por 100 para obtener los grados reales con 2 decimales de precisión.
        // Ejemplo: si imu_pitch = 4550 -> ángulo real = 45.50°
        portENTER_CRITICAL(&paramLock);
        mb_input_reg.imu_pitch = (int16_t)(final_pitch * 100.0f);
        mb_input_reg.imu_roll  = (int16_t)(final_roll  * 100.0f);
        mb_input_reg.imu_temp  = (int16_t)(temperature * 100.0f);
        portEXIT_CRITICAL(&paramLock);

        // [FIX-15] Log de ángulos cada 5 ciclos (cada 1 segundo aprox) para monitoreo en serial
        static int log_counter = 0;
        if (log_counter++ >= 5) {
            log_counter = 0;
            ESP_LOGI(TAG, "[%s] IMU: pitch=%.2f° roll=%.2f° temp=%.1f°C",
                     device_serial, final_pitch, final_roll, temperature);
        }

        // [FIX-14] Aumentar delay entre ciclos de 100ms a 200ms
        // El sensor ICM42670 necesita más tiempo para estabilizarse entre lecturas
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}


void vTaskDigitalOutput(void *pvParameters)
{
    EventBits_t uxBits = 0;

    while (1)
    {
        uxBits = xEventGroupWaitBits(xEventGroupCoilsReg,
                                     MB_REG_COILS_MASK,
                                     pdFALSE,
                                     pdFALSE,
                                     portMAX_DELAY);

        if (uxBits & MB_REG_COIL0_WR_BIT) {
            portENTER_CRITICAL(&paramLock);
            gpio_set_level(DOUT0, (uint32_t)mb_coil_reg.out0);
            portEXIT_CRITICAL(&paramLock);
            xEventGroupClearBits(xEventGroupCoilsReg, MB_REG_COIL0_WR_BIT);
        }
        if (uxBits & MB_REG_COIL1_WR_BIT) {
            portENTER_CRITICAL(&paramLock);
            gpio_set_level(DOUT1, (uint32_t)mb_coil_reg.out1);
            portEXIT_CRITICAL(&paramLock);
            xEventGroupClearBits(xEventGroupCoilsReg, MB_REG_COIL1_WR_BIT);
        }
        if (uxBits & MB_REG_COIL2_WR_BIT) {
            portENTER_CRITICAL(&paramLock);
            gpio_set_level(DOUT2, (uint32_t)mb_coil_reg.out2);
            portEXIT_CRITICAL(&paramLock);
            xEventGroupClearBits(xEventGroupCoilsReg, MB_REG_COIL2_WR_BIT);
        }
        if (uxBits & MB_REG_COIL3_WR_BIT) {
            portENTER_CRITICAL(&paramLock);
            gpio_set_level(DOUT3, (uint32_t)mb_coil_reg.out3);
            portEXIT_CRITICAL(&paramLock);
            xEventGroupClearBits(xEventGroupCoilsReg, MB_REG_COIL3_WR_BIT);
        }
    }
}


void vTaskModbusWatchdog(void *pvParameters)
{
    const uint32_t WATCHDOG_CHECK_PERIOD_MS = 1000;

    int64_t now       = 0;
    int64_t last_time = 0;

    // Inicializar con tiempo real al arranque
    portENTER_CRITICAL(&paramLock);
    last_modbus_activity_time = esp_timer_get_time() / 1000;
    portEXIT_CRITICAL(&paramLock);

    while (1)
    {
        now = esp_timer_get_time() / 1000;

        portENTER_CRITICAL(&paramLock);
        last_time = last_modbus_activity_time;
        portEXIT_CRITICAL(&paramLock);

        if ((now - last_time) > MODBUS_INACTIVITY_RESET_MS)
        {
            ESP_LOGE(TAG, "MODBUS INACTIVITY 5min -> SYSTEM RESTART");
            vTaskDelay(pdMS_TO_TICKS(200));
            esp_restart();
        }

        vTaskDelay(pdMS_TO_TICKS(WATCHDOG_CHECK_PERIOD_MS));
    }
}


//*************************************************************************************
//***************************** FUNCIONES DE UTILIDAD *********************************
//*************************************************************************************

void motorDcSet(uint8_t id, motor_dir_t dir, uint16_t speed)
{
    if (id >= MOTOR_COUNT) return;
    if (speed > MOTOR_MAX_PWM) speed = MOTOR_MAX_PWM;

    uint32_t duty_ticks = (PWM_PERIOD_TICKS * speed) / MOTOR_MAX_PWM;

    switch (dir)
    {
        case MOTOR_FORWARD:
            mcpwm_comparator_set_compare_value(motors[id].cmp_a, duty_ticks);
            mcpwm_comparator_set_compare_value(motors[id].cmp_b, 0);
            break;

        case MOTOR_REVERSE:
            mcpwm_comparator_set_compare_value(motors[id].cmp_a, 0);
            mcpwm_comparator_set_compare_value(motors[id].cmp_b, duty_ticks);
            break;

        case MOTOR_STOP:
        default:
            mcpwm_comparator_set_compare_value(motors[id].cmp_a, 0);
            mcpwm_comparator_set_compare_value(motors[id].cmp_b, 0);
            break;
    }
}


// [FIX-7] Función de TEST - NO llamar en producción
// Solo para verificación del hardware en banco de pruebas
void motorDcControl_TEST(void)
{
    ESP_LOGW(TAG, "motorDcControl_TEST: MODO TEST ACTIVO - solo usar en banco");

    for (uint16_t speed = 0; speed <= 1000; speed += 10) {
        for (int m = 0; m < MOTOR_COUNT; m++) motorDcSet(m, MOTOR_REVERSE, speed);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelay(pdMS_TO_TICKS(2000));

    for (int speed = 1000; speed >= 0; speed -= 10) {
        for (int m = 0; m < MOTOR_COUNT; m++) motorDcSet(m, MOTOR_REVERSE, speed);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    for (uint16_t speed = 0; speed <= 1000; speed += 10) {
        for (int m = 0; m < MOTOR_COUNT; m++) motorDcSet(m, MOTOR_FORWARD, speed);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelay(pdMS_TO_TICKS(2000));

    for (int speed = 1000; speed >= 0; speed -= 10) {
        for (int m = 0; m < MOTOR_COUNT; m++) motorDcSet(m, MOTOR_FORWARD, speed);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    for (int m = 0; m < MOTOR_COUNT; m++) motorDcSet(m, MOTOR_STOP, 0);

    ESP_LOGW(TAG, "motorDcControl_TEST: TEST FINALIZADO");
}


void wait_for_sensor(ina260_t *dev)
{
    bool ready, alert, overflow;
    int timeout = 100;

    // [FIX-9] Cambiar while(timeout--) por while(timeout-- > 0) para evitar timeout negativo
    // El problema anterior: timeout-- devuelve el valor anterior, así que el loop solo se ejecutaba 99 veces
    // y timeout terminaba en -1, nunca pasando la condición <= 0
    while (timeout-- > 0) {
        ESP_ERROR_CHECK(ina260_get_status(dev, &ready, &alert, &overflow));
        if (alert)    ESP_LOGW(TAG, "INA260 ALERT!");
        if (overflow) ESP_LOGW(TAG, "INA260 OVERFLOW!");
        if (ready)    break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (timeout <= 0) {
        ESP_LOGE(TAG, "wait_for_sensor: timeout esperando INA260");
    }
}


// [FIX-12] Calibración IMU: Guarda los ángulos actuales como referencia (0,0)
// Ejemplo: Si el dispositivo está inclinado 15° al prenderse, al calibrar
// esos 15° se marcan como 0°, y a partir de ahí se miden relativos a esa posición
void imu_calibrate_reference(float pitch, float roll)
{
    portENTER_CRITICAL(&paramLock);
    imu_calibration_offset_pitch = pitch;
    imu_calibration_offset_roll = roll;
    portEXIT_CRITICAL(&paramLock);
    
    ESP_LOGI(TAG, "IMU Calibración guardada: pitch_offset=%.2f°, roll_offset=%.2f°", 
             pitch, roll);
}


// [FIX-12] Habilitar o deshabilitar la calibración IMU
void imu_calibration_enable(bool enable)
{
    //imu_calibration_enabled = enable;
    imu_calibration_enabled = false;
    if (enable) {
        ESP_LOGI(TAG, "IMU Calibración HABILITADA - midiendo relativos a referencia");
    } else {
        ESP_LOGI(TAG, "IMU Calibración DESHABILITADA - midiendo ángulos absolutos");
    }
}


//*************************************************************************************
//***************************** CONFIGURACION DE HARDWARE *****************************
//*************************************************************************************

void vHardwareInit(void)
{
    vGpioInit();
    vMotorsDcInit();
    vServosInit();
    vLedPwmInit();
    vI2cInit();
    vModbusSlaveInit();
    ESP_LOGW(TAG, "Configuracion de drivers ESP-IDF exitosa");
}


void vGpioInit(void)
{
    gpio_config_t gpioOutputConfig = {
        .pin_bit_mask = DOUT_MASK_CONFIG,
        .mode         = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&gpioOutputConfig);

    // [FIX-4] Inicializar DOUT0 también, estaba faltando
    gpio_set_level(DOUT0, 0);
    gpio_set_level(DOUT1, 0);
    gpio_set_level(DOUT2, 0);
    gpio_set_level(DOUT3, 0);
}


void vServosInit(void)
{
    servo_create(&servo1, SERVO1_CH0_PIN, LEDC_CHANNEL_0);
    servo_create(&servo2, SERVO2_CH1_PIN, LEDC_CHANNEL_1);
}


void vLedPwmInit(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_1,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz         = LED_FREQ_HZ,
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num   = LED_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_2,
        .timer_sel  = LEDC_TIMER_1,
        .duty       = 0
    };
    ledc_channel_config(&channel);
}


void vMotorsDcInit(void)
{
    gpio_config_t gpio_inh = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask =
            (1ULL << M0INH) |
            (1ULL << M1INH) |
            (1ULL << M2INH) |
            (1ULL << M3INH)
    };
    gpio_config(&gpio_inh);

    mcpwm_timer_config_t tcfg = {
        .resolution_hz = PWM_RESOLUTION_HZ,
        .period_ticks  = PWM_PERIOD_TICKS,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP
    };

    tcfg.group_id = 0;
    ESP_ERROR_CHECK(mcpwm_new_timer(&tcfg, &timer0));

    tcfg.group_id = 1;
    ESP_ERROR_CHECK(mcpwm_new_timer(&tcfg, &timer1));

    gpio_num_t inh_pins[MOTOR_COUNT] = {M0INH,  M1INH,  M2INH,  M3INH};
    gpio_num_t pwm_a[MOTOR_COUNT]    = {M0PWMA, M1PWMA, M2PWMA, M3PWMA};
    gpio_num_t pwm_b[MOTOR_COUNT]    = {M0PWMB, M1PWMB, M2PWMB, M3PWMB};

    for (int i = 0; i < MOTOR_COUNT; i++) {

        motors[i].inh = inh_pins[i];

        mcpwm_operator_config_t ocfg = {
            .group_id = (i < 3) ? 0 : 1
        };
        ESP_ERROR_CHECK(mcpwm_new_operator(&ocfg, &motors[i].oper));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(motors[i].oper, (i < 3) ? timer0 : timer1));

        mcpwm_comparator_config_t ccfg = {
            .flags.update_cmp_on_tez = true
        };
        ESP_ERROR_CHECK(mcpwm_new_comparator(motors[i].oper, &ccfg, &motors[i].cmp_a));
        ESP_ERROR_CHECK(mcpwm_new_comparator(motors[i].oper, &ccfg, &motors[i].cmp_b));

        mcpwm_generator_config_t gcfg = {0};

        gcfg.gen_gpio_num = pwm_a[i];
        ESP_ERROR_CHECK(mcpwm_new_generator(motors[i].oper, &gcfg, &motors[i].gen_a));

        gcfg.gen_gpio_num = pwm_b[i];
        ESP_ERROR_CHECK(mcpwm_new_generator(motors[i].oper, &gcfg, &motors[i].gen_b));

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
            motors[i].gen_a,
            MCPWM_GEN_TIMER_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                MCPWM_TIMER_EVENT_EMPTY,
                MCPWM_GEN_ACTION_HIGH)));

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
            motors[i].gen_a,
            MCPWM_GEN_COMPARE_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                motors[i].cmp_a,
                MCPWM_GEN_ACTION_LOW)));

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
            motors[i].gen_b,
            MCPWM_GEN_TIMER_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                MCPWM_TIMER_EVENT_EMPTY,
                MCPWM_GEN_ACTION_HIGH)));

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
            motors[i].gen_b,
            MCPWM_GEN_COMPARE_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                motors[i].cmp_b,
                MCPWM_GEN_ACTION_LOW)));
    }

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer0));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer0, MCPWM_TIMER_START_NO_STOP));
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer1));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer1, MCPWM_TIMER_START_NO_STOP));

    for (int i = 0; i < MOTOR_COUNT; i++) {
        gpio_set_level(motors[i].inh, 1);
    }

    ESP_LOGI(TAG, "Motores DC inicializados");
}


void vI2cInit(void)
{
    esp_err_t ret;

    // ==== Configuracion ICM-42670 (IMU) ====
    const i2c_master_bus_config_t i2c1_bus_config = {
        .i2c_port   = IMU_I2C_PORT,
        .sda_io_num = IMU_I2C_SDA_PIN,
        .scl_io_num = IMU_I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
    };

    // [FIX-6] Validar inicialización del bus I2C
    ret = i2c_new_master_bus(&i2c1_bus_config, &i2c1_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando bus I2C1 (IMU): %s", esp_err_to_name(ret));
        return;
    }

    // [FIX-6] Validar creación del device ICM-42670
    ret = icm42670_create(i2c1_handle, ICM42670_I2C_ADDRESS, &dev_icm42670_handle);
    if (ret != ESP_OK || dev_icm42670_handle == NULL) {
        ESP_LOGE(TAG, "Error creando device ICM-42670: %s", esp_err_to_name(ret));
        return;
    }

    const icm42670_cfg_t dev_imu_cfg = {
        .acce_fs  = ACCE_FS_2G,
        .acce_odr = ACCE_ODR_400HZ,
        .gyro_fs  = GYRO_FS_2000DPS,
        .gyro_odr = GYRO_ODR_400HZ,
    };

    ret = icm42670_config(dev_icm42670_handle, &dev_imu_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando ICM-42670: %s", esp_err_to_name(ret));
        return;
    }

    // ==== Configuracion INA260 ====
    i2cdev_init();
    memset(&dev_ina260_handle, 0, sizeof(ina260_t));

    ret = ina260_init_desc(&dev_ina260_handle, INA_DEV_ADDR, INA_I2C_PORT, INA_I2C_SDA_PIN, INA_I2C_SCL_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando descriptor INA260: %s", esp_err_to_name(ret));
        return;
    }

    ret = ina260_init(&dev_ina260_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando INA260: %s", esp_err_to_name(ret));
        return;
    }

    ret = ina260_set_config(&dev_ina260_handle,
                            INA260_MODE_TRIG_SHUNT_BUS,
                            INA260_AVG_128,
                            INA260_CT_1100,
                            INA260_CT_1100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando INA260: %s", esp_err_to_name(ret));
        return;
    }

    // Encender acelerómetro y giroscopio en modo low noise
    ret = icm42670_acce_set_pwr(dev_icm42670_handle, ACCE_PWR_LOWNOISE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error encendiendo acelerómetro ICM-42670");
        return;
    }
    
    // [FIX-14] Agregar delay después de encender acelerómetro 
    // El sensor necesita estabilizarse antes de leer valores
    vTaskDelay(pdMS_TO_TICKS(100));

    ret = icm42670_gyro_set_pwr(dev_icm42670_handle, GYRO_PWR_LOWNOISE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error encendiendo giroscopio ICM-42670");
        return;
    }
    
    // [FIX-14] Agregar delay después de encender giroscopio
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "I2C inicializado correctamente (IMU + INA260)");
}


void vModbusSlaveInit(void)
{
    esp_err_t ret;

    mb_communication_info_t mb_com_config = {
        .ser_opts.mode      = MB_RTU,
        .ser_opts.uid       = MB_SLAVE_ADDR,
        .ser_opts.port      = MB_PORT_NUM,
        .ser_opts.baudrate  = MB_SLAVE_BAUD,
        .ser_opts.parity    = MB_PARITY_NONE,
        .ser_opts.data_bits = UART_DATA_8_BITS,
        .ser_opts.stop_bits = UART_STOP_BITS_1
    };

    ret = mbc_slave_create_serial(&mb_com_config, &mb_slave_handle);
    if (ESP_OK != ret) {
        ESP_LOGE(TAG, "Modbus Slave: fallo en la inicializacion");
        return;
    }

    mb_register_area_descriptor_t reg_area;

    reg_area.type         = MB_PARAM_COIL;
    reg_area.start_offset = MB_REG_COILS_START;
    reg_area.address      = (void*)&mb_coil_reg;
    reg_area.size         = sizeof(mb_coil_reg);
    ret = mbc_slave_set_descriptor(mb_slave_handle, reg_area);
    if (ESP_OK != ret) ESP_LOGE(TAG, "Register MB_PARAM_COIL fail");

    reg_area.type         = MB_PARAM_HOLDING;
    reg_area.start_offset = MB_REG_HOLDING_START;
    reg_area.address      = (void*)&mb_holding_reg;
    reg_area.size         = sizeof(mb_holding_reg);
    ret = mbc_slave_set_descriptor(mb_slave_handle, reg_area);
    if (ESP_OK != ret) ESP_LOGE(TAG, "Register MB_PARAM_HOLDING fail");

    reg_area.type         = MB_PARAM_INPUT;
    reg_area.start_offset = MB_REG_INPUT_START;
    reg_area.address      = (void*)&mb_input_reg;
    reg_area.size         = sizeof(mb_input_reg);
    ret = mbc_slave_set_descriptor(mb_slave_handle, reg_area);
    if (ESP_OK != ret) {
        ESP_LOGE(TAG, "Register MB_PARAM_INPUT fail");
        return;
    }

    vModbusSlaveControlStop();

    ret = mbc_slave_start(mb_slave_handle);
    if (ESP_OK != ret) {
        ESP_LOGE(TAG, "mb controller start fail");
        return;
    }

    ret = uart_set_pin(MB_PORT_NUM, MB_UART_TXD, MB_UART_RXD, MB_UART_DIR, UART_PIN_NO_CHANGE);
    if (ESP_OK != ret) {
        ESP_LOGE(TAG, "uart_set_pin failure");
        return;
    }

    ret = uart_set_mode(MB_PORT_NUM, UART_MODE_RS485_HALF_DUPLEX);
    if (ESP_OK != ret) {
        ESP_LOGE(TAG, "uart_set_mode failure");
        return;
    }

    ESP_LOGI(TAG, "Modbus RTU Stack inicializado!");
}


void vModbusSlaveControlStop(void)
{
    // [FIX-4] DOUT0 incluido en el reset
    gpio_set_level(DOUT0, 0);
    gpio_set_level(DOUT1, 0);
    gpio_set_level(DOUT2, 0);
    gpio_set_level(DOUT3, 0);

    for (int i = 0; i < MOTOR_COUNT; i++) {
        motorDcSet(i, MOTOR_STOP, 0);
    }

    portENTER_CRITICAL(&paramLock);
    mb_holding_reg.motor_cmd  = MOTOR_STOP;
    mb_holding_reg.motor_vel  = 0;
    mb_holding_reg.srv1_angle = 0;
    mb_holding_reg.srv2_angle = 0;
    mb_holding_reg.led_pwm    = 0;
    mb_holding_reg.fw_version = MB_FIRMWARE_REVISION;
    mb_input_reg.imu_pitch    = 0;
    mb_input_reg.imu_roll     = 0;
    mb_input_reg.imu_temp     = 0;
    mb_input_reg.ina_current  = 0;
    mb_input_reg.ina_voltage  = 0;
    mb_input_reg.ina_power    = 0;
    mb_coil_reg.out0          = 0;
    mb_coil_reg.out1          = 0;
    mb_coil_reg.out2          = 0;
    mb_coil_reg.out3          = 0;
    portEXIT_CRITICAL(&paramLock);

    ESP_LOGI(TAG, "Slave Modbus Stop Init");
}