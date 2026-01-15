/***********************************************************************
 * PROJECT: DEFENSE-GSE-77 (Ground Support Equipment)
 * SYSTEM:  Sector Scan & Perimeter Security Controller (FCC)
 * * CONTEXT:
 * This system controls a high-kinetic "Sector Scan" radar/turret mechanism.
 * It enforces a strict 2-meter Personnel Exclusion Zone to prevent
 * mechanical crush injuries and RF radiation exposure.
 * * * SAFETY ARCHITECTURE (MIL-STD-882E Inspired):
 * - HARDWARE INTERLOCK (Category 1): Master Arm Switch (Kill Switch).
 * - SOFTWARE INTERLOCK (Category 2): RSO Reset Interface.
 * - SENSOR INTERLOCK (Category 3): Exclusion Zone Monitor.
 * - AUDIBLE ALARM: Klaxon triggers ONLY on Exclusion Zone Breach.
 * * * LOGIC:
 * 1. ARM: Master Arm Switch must be ON (Closed).
 * 2. RESET: RSO must press Button to clear "Latched Fault" states.
 * 3. TRIGGER: Sensor Breach (<200cm) causes IMMEDIATE HALT + KLAXON.
 ***********************************************************************/

#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"

/* ===================== GPIO ASSIGNMENTS ===================== */

// Status Indicators (Standard GSE Colors)
#define LED_SYSTEM_HEARTBEAT      GPIO_NUM_5   // Green: CPU/Scheduler Alive
#define LED_FAULT_INDICATOR       GPIO_NUM_4   // Red: Fault / Safety Lock
#define LED_ARMED_STATUS          GPIO_NUM_19  // Blue: Active / Scanning

// Operator Interface
#define SWITCH_MASTER_ARM         GPIO_NUM_18  // Slide Switch (Hardware Kill)
#define BUTTON_RSO_RESET          GPIO_NUM_23  // Momentary Button (Acknowledge)
#define BUZZER_KLAXON_PIN         GPIO_NUM_21  // Piezo Alarm

// Sensors
#define RADAR_TRIG_PIN            GPIO_NUM_17  
#define RADAR_ECHO_PIN            GPIO_NUM_16

// Actuator (Sector Scan Mechanism)
#define ACTUATOR_SERVO_PIN        GPIO_NUM_13  

/* ===================== SYSTEM CONSTANTS ===================== */

#define SAFETY_PERIMETER_CM           200     // 2 Meters (Stand-off Distance)
#define DEBOUNCE_TIME_MS              200     
#define SENSOR_TIMEOUT_US           30000     // 30ms Watchdog

// Servo Config (50Hz PWM)
#define SERVO_TIMER              LEDC_TIMER_0
#define SERVO_MODE               LEDC_LOW_SPEED_MODE
#define SERVO_CHANNEL            LEDC_CHANNEL_0
#define SERVO_DUTY_RES           LEDC_TIMER_13_BIT 
#define SERVO_FREQUENCY          50                

/* ===================== RTOS OBJECTS ===================== */

SemaphoreHandle_t sem_reset_button;     
SemaphoreHandle_t sem_perimeter_breach;

/* ===================== STATE MACHINE ===================== */

typedef enum {
    SYS_OPERATIONAL,       // Active Scanning
    SYS_HALTED_FOD,        // BREACH -> Halt + Klaxon
    SYS_HALTED_RSO,        // Master Arm OFF (Safe)
    SYS_AWAITING_RESET     // Waiting for RSO Confirmation
} SystemState;

volatile SystemState system_state = SYS_HALTED_RSO; // Default to Safe State
volatile bool is_perimeter_fouled = false;
volatile int current_range_cm = -1;

/* ===================== FUNCTION PROTOTYPES ===================== */

void task_heartbeat(void *pvParameters);
void task_perimeter_radar(void *pvParameters);
void task_actuator_control(void *pvParameters); 
void task_telemetry_out(void *pvParameters);
void task_buzzer_alarm(void *pvParameters);
static void isr_reset_button(void *arg); 
void init_actuator_servo(void);
void set_actuator_angle(int angle);

/* ===================== MAIN ENTRY ===================== */

void app_main(void)
{
    /* 1. Configure LEDs */
    gpio_config_t led_cfg = {
        .pin_bit_mask = (1ULL << LED_SYSTEM_HEARTBEAT) |
                        (1ULL << LED_FAULT_INDICATOR) |
                        (1ULL << LED_ARMED_STATUS),
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&led_cfg);

    /* 2. Configure Klaxon */
    gpio_config_t buzz_cfg = {
        .pin_bit_mask = (1ULL << BUZZER_KLAXON_PIN),
        .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&buzz_cfg);

    /* 3. Configure Sensors */
    gpio_set_direction(RADAR_TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(RADAR_ECHO_PIN, GPIO_MODE_INPUT);

    /* 4. Configure Master Arm (State Input) */
    gpio_config_t sw_cfg = {
        .pin_bit_mask = (1ULL << SWITCH_MASTER_ARM),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE 
    };
    gpio_config(&sw_cfg);

    /* 5. Configure RSO Reset (Interrupt Input) */
    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << BUTTON_RSO_RESET),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&btn_cfg);

    /* 6. Init Actuator */
    init_actuator_servo();

    /* 7. Create RTOS Objects */
    sem_reset_button = xSemaphoreCreateBinary();
    sem_perimeter_breach = xSemaphoreCreateBinary();

    /* 8. Launch Tasks (Priority-Driven) */
    xTaskCreate(task_telemetry_out, "Telemetry", 2048, NULL, 1, NULL);
    xTaskCreate(task_heartbeat,     "Heartbeat", 2048, NULL, 1, NULL);
    xTaskCreate(task_perimeter_radar, "Radar",   2048, NULL, 2, NULL);
    xTaskCreate(task_actuator_control,"Actuator",2048, NULL, 3, NULL);
    xTaskCreate(task_buzzer_alarm,    "Klaxon",  2048, NULL, 2, NULL);

    /* 9. Install ISR */
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_RSO_RESET, isr_reset_button, NULL);
}

/* ===================== ACTUATOR DRIVER ===================== */

void init_actuator_servo(void) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = SERVO_MODE,
        .timer_num        = SERVO_TIMER,
        .duty_resolution  = SERVO_DUTY_RES,
        .freq_hz          = SERVO_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = SERVO_MODE,
        .channel        = SERVO_CHANNEL,
        .timer_sel      = SERVO_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = ACTUATOR_SERVO_PIN,
        .duty           = 0, 
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
}

void set_actuator_angle(int angle) {
    // Map 0-180 degrees to pulse width for standard servo
    uint32_t duty = (uint32_t)(205 + (angle * (983 - 205) / 180));
    ledc_set_duty(SERVO_MODE, SERVO_CHANNEL, duty);
    ledc_update_duty(SERVO_MODE, SERVO_CHANNEL);
}

/* ===================== INTERRUPT SERVICE ROUTINE ===================== */

static void IRAM_ATTR isr_reset_button(void *arg)
{
    static int64_t last_time = 0;
    int64_t now = esp_timer_get_time();

    // Software Debounce (200ms)
    if ((now - last_time) > (DEBOUNCE_TIME_MS * 1000)) {
        last_time = now;
        xSemaphoreGiveFromISR(sem_reset_button, NULL);
        portYIELD_FROM_ISR();
    }
}

/* ===================== CRITICAL CONTROL TASK ===================== */

void task_actuator_control(void *pvParameters)
{
    int actuator_pos = 0;
    int direction = 1;
    const int step_size = 2; 

    while (1) {
        
        bool is_armed = gpio_get_level(SWITCH_MASTER_ARM); 

        if (!is_armed) {
            // --- STATE: HARD KILL (Master Arm OFF) ---
            system_state = SYS_HALTED_RSO; 
            gpio_set_level(LED_FAULT_INDICATOR, 1); 
            gpio_set_level(LED_ARMED_STATUS, 0);    
            
            /* Input Sanitization:
             * Drain the semaphore buffer to discard any button presses 
             * made while the system was in the Safe/Kill state.
             * This prevents uncommanded restarts upon re-arming. */
            xSemaphoreTake(sem_reset_button, 0); 
        }
        else {
            // --- STATE: ARMED (Master Arm ON) ---
            
            // 1. Transition Check: If we just woke up, enter Lockout
            if (system_state == SYS_HALTED_RSO) {
                 system_state = SYS_AWAITING_RESET;
            }

            // 2. Sensor Check (Highest Priority Fault)
            if (xSemaphoreTake(sem_perimeter_breach, 0)) {
                system_state = SYS_HALTED_FOD;
                gpio_set_level(LED_FAULT_INDICATOR, 1);
                gpio_set_level(LED_ARMED_STATUS, 0);
            }

            // 3. RSO Reset Check (Only valid if Armed)
            if (xSemaphoreTake(sem_reset_button, 0)) {
                // Two-Condition Interlock:
                // Reset is only accepted if we are in a Fault/Wait state
                // AND the sensor path is clear.
                if ((system_state == SYS_HALTED_FOD || system_state == SYS_AWAITING_RESET) 
                    && !is_perimeter_fouled) {
                    
                    system_state = SYS_OPERATIONAL;
                    gpio_set_level(LED_FAULT_INDICATOR, 0);
                    gpio_set_level(LED_ARMED_STATUS, 1);
                }
            }
            
            // 4. Auto-clearance Check (FOD Removed -> Wait for Reset)
            if (system_state == SYS_HALTED_FOD && !is_perimeter_fouled) {
                system_state = SYS_AWAITING_RESET;
            }
        }

        // --- MOTOR LOGIC ---
        if (system_state == SYS_OPERATIONAL && is_armed) {
            actuator_pos += (step_size * direction);
            if (actuator_pos >= 180) { actuator_pos = 180; direction = -1; }
            else if (actuator_pos <= 0) { actuator_pos = 0; direction = 1; }
            set_actuator_angle(actuator_pos);
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // Servo Update Rate
    }
}

/* ===================== KLAXON ALARM TASK ===================== */
/* Generates a 1kHz tone ONLY when Exclusion Zone is breached */

void task_buzzer_alarm(void *pvParameters)
{
    while (1) {
        if (system_state == SYS_HALTED_FOD) {
            // Generate 1kHz Square Wave
            gpio_set_level(BUZZER_KLAXON_PIN, 1);
            ets_delay_us(500); 
            gpio_set_level(BUZZER_KLAXON_PIN, 0);
            ets_delay_us(500);
        } else {
            // Silence & Sleep
            gpio_set_level(BUZZER_KLAXON_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(100)); 
        }
    }
}

/* ===================== EXCLUSION ZONE MONITOR ===================== */

void task_perimeter_radar(void *pvParameters)
{
    bool prev_breach = false;

    while (1) {
        // Trigger Pulse
        gpio_set_level(RADAR_TRIG_PIN, 0);
        ets_delay_us(2);
        gpio_set_level(RADAR_TRIG_PIN, 1);
        ets_delay_us(10);
        gpio_set_level(RADAR_TRIG_PIN, 0);

        // Wait for Echo Start (Watchdog protected)
        int64_t start = esp_timer_get_time();
        while (!gpio_get_level(RADAR_ECHO_PIN)) {
            if ((esp_timer_get_time() - start) > SENSOR_TIMEOUT_US) {
                is_perimeter_fouled = true; // Fail-safe: Assume breach
                current_range_cm = -1; 
                goto radar_sleep;
            }
        }

        // Measure Pulse Width
        int64_t echo_start = esp_timer_get_time();
        while (gpio_get_level(RADAR_ECHO_PIN)) {
            if ((esp_timer_get_time() - echo_start) > SENSOR_TIMEOUT_US) {
                is_perimeter_fouled = true; // Fail-safe: Assume breach
                current_range_cm = -1;
                goto radar_sleep;
            }
        }
        int64_t echo_end = esp_timer_get_time();

        int distance = (int)((echo_end - echo_start) * 0.0343 / 2.0);
        current_range_cm = distance;

        // Check Exclusion Zone
        bool breach_now = (distance > 0 && distance < SAFETY_PERIMETER_CM);
        is_perimeter_fouled = breach_now;

        // Signal Control Task on NEW breach only
        if (breach_now && !prev_breach) {
            xSemaphoreGive(sem_perimeter_breach);
        }
        prev_breach = breach_now;

radar_sleep:
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ===================== TELEMETRY OUTPUT ===================== */

void task_telemetry_out(void *pvParameters)
{
    char *state_str;
    while (1) {
        switch (system_state) {
            case SYS_OPERATIONAL:    state_str = "STATUS: ARMED - ACTIVE - DANGER! KEEP CLEAR 2M"; break;
            case SYS_HALTED_FOD:     state_str = "FAULT: BREACH DETECTED - SHUT DOWN"; break;
            case SYS_HALTED_RSO:     state_str = "FAULT: MASTER ARM SWITCH OFF"; break;
            case SYS_AWAITING_RESET: state_str = "LOCK: EXCLUSION ZONE CLEAR - AWAITING RESET CONFIRM"; break;
            default:                 state_str = "UNKNOWN"; break;
        }

        printf("[T+%lu] Range: %03dcm | %s\n",
               xTaskGetTickCount()/100,
               current_range_cm,
               state_str);

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void task_heartbeat(void *pvParameters)
{
    while (1) {
        gpio_set_level(LED_SYSTEM_HEARTBEAT, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(LED_SYSTEM_HEARTBEAT, 0);
        vTaskDelay(pdMS_TO_TICKS(1500)); 
    }
}