/***********************************************************************
 * Implements a safety-interlocked control system using FreeRTOS.
 * The system halts operation when an obstruction is detected or when
 * an emergency stop is pressed, and only allows restart when:
 *   1) The obstruction is cleared, AND
 *   2) A human operator explicitly confirms restart
 * This architecture mirrors real industrial safety systems:
 *  - Hardware removes energy
 *  - Software enforces restart conditions
 ***********************************************************************/

#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"

/* ===================== GPIO ASSIGNMENTS ===================== */

// Indicator LEDs
#define LED_SYSTEM_POWER          GPIO_NUM_5   // Heartbeat indicator
#define LED_EMERGENCY_BRAKE       GPIO_NUM_4   // Fault / brake engaged
#define LED_ALL_CLEAR             GPIO_NUM_19  // Ready-to-run indicator

// Human interface
#define BUTTON_EMERGENCY_STOP     GPIO_NUM_18  // Normally-closed E-Stop button

// Ultrasonic proximity sensor (HC-SR04 style)
#define PROX_TRIG_PIN             GPIO_NUM_17
#define PROX_ECHO_PIN             GPIO_NUM_16

/* ===================== SYSTEM CONSTANTS ===================== */

#define PROXIMITY_THRESHOLD_CM        30      // Unsafe distance threshold
#define BUTTON_DEBOUNCE_TIME_MS       200     // Debounce window for E-Stop
#define PROX_ECHO_TIMEOUT_US      30000       // ~5 meters max echo time

/* ===================== RTOS OBJECTS ===================== */

SemaphoreHandle_t sem_emergency_stop;
SemaphoreHandle_t sem_proximity_event;

/* ===================== SYSTEM STATE ===================== */

/*
 * RideStatus represents the global operational state of the system.
 * This is effectively a software safety relay.
 */
typedef enum {
    RIDE_ALL_CLEAR,        // Ride may operate
    HALTED_BY_PROXIMITY,   // Automatic safety stop
    HALTED_BY_ESTOP,       // Manual emergency stop
    AWAITING_RESTART       // Obstruction cleared, waiting for operator
} RideStatus;

/*
 * Volatile is critical here: these variables are shared between tasks
 * and ISRs, and must never be cached by the compiler.
 */
volatile RideStatus ride_status = RIDE_ALL_CLEAR;
volatile bool is_obstruction_present = false;
volatile int current_proximity_cm = -1;
volatile int64_t last_estop_isr_time_us = 0;

/* ===================== FUNCTION PROTOTYPES ===================== */

void system_power_monitor_task(void *pvParameters);
void proximity_sensor_task(void *pvParameters);
void ride_control_task(void *pvParameters);
void status_output_task(void *pvParameters);
static void IRAM_ATTR emergency_stop_isr(void *arg);

/* ===================== MAIN APPLICATION ===================== */

void app_main(void)
{
    /* Configure LED outputs */
    gpio_config_t led_cfg = {
        .pin_bit_mask = (1ULL << LED_SYSTEM_POWER) |
                        (1ULL << LED_EMERGENCY_BRAKE) |
                        (1ULL << LED_ALL_CLEAR),
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&led_cfg);

    /* Configure E-Stop input (normally-closed, falling edge interrupt) */
    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << BUTTON_EMERGENCY_STOP),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&btn_cfg);

    /* Configure ultrasonic sensor pins */
    gpio_set_direction(PROX_TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(PROX_ECHO_PIN, GPIO_MODE_INPUT);

    /* Initial safe state */
    gpio_set_level(LED_EMERGENCY_BRAKE, 0);
    gpio_set_level(LED_ALL_CLEAR, 1);

    /* Create synchronization primitives */
    sem_emergency_stop = xSemaphoreCreateBinary();
    sem_proximity_event = xSemaphoreCreateBinary();

    /* Create tasks */
    xTaskCreate(system_power_monitor_task, "PowerLED", 2048, NULL, 1, NULL);
    xTaskCreate(proximity_sensor_task,    "Proximity", 2048, NULL, 2, NULL);
    xTaskCreate(ride_control_task,         "RideCtrl",  2048, NULL, 3, NULL);
    xTaskCreate(status_output_task,        "Status",    2048, NULL, 1, NULL);

    /* Install ISR service and register E-Stop ISR */
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_EMERGENCY_STOP, emergency_stop_isr, NULL);
}

/* ===================== EMERGENCY STOP ISR ===================== */

/*
 * ISR is intentionally minimal:
 *  - Debounce
 *  - Signal the control task
 *  - Yield if required
 */
static void IRAM_ATTR emergency_stop_isr(void *arg)
{
    int64_t now_us = esp_timer_get_time();

    if ((now_us - last_estop_isr_time_us) >
        (BUTTON_DEBOUNCE_TIME_MS * 1000)) {

        last_estop_isr_time_us = now_us;

        BaseType_t task_woken = pdFALSE;
        xSemaphoreGiveFromISR(sem_emergency_stop, &task_woken);

        if (task_woken) {
            portYIELD_FROM_ISR();
        }
    }
}

/* ===================== PROXIMITY SENSOR TASK ===================== */

/*
 * Hard real-time task:
 *  - Measures ultrasonic echo timing
 *  - Applies timeout protection
 *  - Generates a one-shot event on unsafe entry
 */
void proximity_sensor_task(void *pvParameters)
{
    bool prev_obstruction = false;

    while (1) {
        /* Trigger ultrasonic pulse */
        gpio_set_level(PROX_TRIG_PIN, 0);
        ets_delay_us(2);
        gpio_set_level(PROX_TRIG_PIN, 1);
        ets_delay_us(10);
        gpio_set_level(PROX_TRIG_PIN, 0);

        /* Wait for echo rising edge with timeout */
        int64_t start_wait = esp_timer_get_time();
        while (!gpio_get_level(PROX_ECHO_PIN)) {
            if ((esp_timer_get_time() - start_wait) > PROX_ECHO_TIMEOUT_US) {
                /* Sensor failure is treated as unsafe */
                is_obstruction_present = true;
                current_proximity_cm = -1;
                goto sensor_delay;
            }
        }

        /* Measure echo high time */
        int64_t echo_start = esp_timer_get_time();
        while (gpio_get_level(PROX_ECHO_PIN)) {
            if ((esp_timer_get_time() - echo_start) > PROX_ECHO_TIMEOUT_US) {
                is_obstruction_present = true;
                current_proximity_cm = -1;
                goto sensor_delay;
            }
        }

        int64_t echo_end = esp_timer_get_time();

        int duration_us = (int)(echo_end - echo_start);
        int distance_cm = (int)(duration_us * 0.0343f / 2.0f);

        current_proximity_cm = distance_cm;
        bool obstruction_now = (distance_cm > 0 &&
                                distance_cm < PROXIMITY_THRESHOLD_CM);

        is_obstruction_present = obstruction_now;

        /* Generate event only on unsafe entry */
        if (obstruction_now && !prev_obstruction) {
            xSemaphoreGive(sem_proximity_event);
        }

        prev_obstruction = obstruction_now;

sensor_delay:
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ===================== RIDE CONTROL TASK ===================== */

/*
 * Highest-priority logic task.
 * Enforces the safety state machine and restart rules.
 */
void ride_control_task(void *pvParameters)
{
    while (1) {

        /* Proximity-triggered halt */
        if ((ride_status == RIDE_ALL_CLEAR ||
             ride_status == AWAITING_RESTART) &&
            xSemaphoreTake(sem_proximity_event, 0)) {

            ride_status = HALTED_BY_PROXIMITY;
            gpio_set_level(LED_EMERGENCY_BRAKE, 1);
            gpio_set_level(LED_ALL_CLEAR, 0);
        }

        /* Emergency stop handling */
        if (xSemaphoreTake(sem_emergency_stop, 0)) {
            switch (ride_status) {

                case RIDE_ALL_CLEAR:
                case HALTED_BY_PROXIMITY:
                    ride_status = HALTED_BY_ESTOP;
                    gpio_set_level(LED_EMERGENCY_BRAKE, 1);
                    gpio_set_level(LED_ALL_CLEAR, 0);
                    break;

                case HALTED_BY_ESTOP:
                case AWAITING_RESTART:
                    if (!is_obstruction_present) {
                        ride_status = RIDE_ALL_CLEAR;
                        gpio_set_level(LED_EMERGENCY_BRAKE, 0);
                        gpio_set_level(LED_ALL_CLEAR, 1);
                    }
                    break;
            }
        }

        /* Automatic transition when obstruction clears */
        if (ride_status == HALTED_BY_PROXIMITY &&
            !is_obstruction_present) {
            ride_status = AWAITING_RESTART;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ===================== STATUS OUTPUT TASK ===================== */

/*
 * Soft real-time diagnostic output.
 * This task must never affect safety behavior.
 */
void status_output_task(void *pvParameters)
{
    while (1) {
        const char *status;

        switch (ride_status) {
            case HALTED_BY_PROXIMITY:
                status = "Obstruction Detected - Ride Halted";
                break;
            case HALTED_BY_ESTOP:
                status = "Emergency Stop Activated";
                break;
            case AWAITING_RESTART:
                status = "Clear - Awaiting Operator Restart";
                break;
            default:
                status = "All Clear";
                break;
        }

        printf("[%lu] Proximity=%dcm | State=%s\n",
               xTaskGetTickCount(),
               current_proximity_cm,
               status);

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

/* ===================== POWER LED TASK ===================== */

/*
 * Lowest-priority heartbeat indicator.
 * Confirms system liveness.
 */
void system_power_monitor_task(void *pvParameters)
{
    while (1) {
        gpio_set_level(LED_SYSTEM_POWER, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_set_level(LED_SYSTEM_POWER, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
