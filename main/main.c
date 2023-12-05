#include <driver/adc.h>
#include <driver/gpio.h>
#include <driver/mcpwm_prelude.h>
#include <esp_adc_cal.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <stdlib.h>

#define DEFAULT_VREF  4096
#define NO_OF_SAMPLES 64

#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500 // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        -90  // Minimum angle
#define SERVO_MAX_DEGREE        90   // Maximum angle

#define SERVO_PULSE_GPIO             2
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000   // 20000 ticks, 20ms

#define NORET __attribute__((noreturn))

typedef uint8_t ui8;
typedef int8_t  i8;

typedef uint16_t ui16;
typedef int16_t  i16;

typedef uint32_t u32;
typedef int32_t  i32;

typedef float  f32;
typedef double f64;

typedef unsigned char byte;

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t            channel = ADC_CHANNEL_3;
static const adc_bits_width_t         width   = ADC_WIDTH_BIT_12;
static const adc_atten_t              atten   = ADC_ATTEN_DB_0;
static const adc_unit_t               unit    = ADC_UNIT_1;

static inline u32 angle_to_compare(i32 angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) /
               (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) +
           SERVO_MIN_PULSEWIDTH_US;
}

i32 mapv(u32 input)
{
    if (input < 0 || input > 4096)
    {
        printf("Input value out of range (0-4096)\n");
        return 0;
    }

    f64 mapped = ((f64)input / 4096.0) * 180.0 - 90.0;
    return (i32)mapped;
}

void app_main(void)
{
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
        printf("eFuse Two Point: Supported\n");
    else
        printf("eFuse Two Point: NOT supported\n");
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)
        printf("eFuse Vref: Supported\n");
    else
        printf("eFuse Vref: NOT supported\n");
    adc2_config_channel_atten((adc2_channel_t)channel, atten);

    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);

    mcpwm_timer_handle_t timer        = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id      = 0,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks  = SERVO_TIMEBASE_PERIOD,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t     oper            = NULL;
    mcpwm_operator_config_t operator_config = {.group_id = 0};
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    mcpwm_cmpr_handle_t       comparator        = NULL;
    mcpwm_comparator_config_t comparator_config = {.flags.update_cmp_on_tez = true};
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    mcpwm_gen_handle_t       generator        = NULL;
    mcpwm_generator_config_t generator_config = {.gen_gpio_num = SERVO_PULSE_GPIO};
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angle_to_compare(0)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        generator, MCPWM_GEN_TIMER_EVENT_ACTION(
                       MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        generator, MCPWM_GEN_COMPARE_EVENT_ACTION(
                       MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    i32 angle = 0;
    i32 step  = 25;

    for (;;)
    {
        u32 adc_reading = 0;
        for (int i = 0; i < NO_OF_SAMPLES; i++)
        {
            i32 raw;
            adc2_get_raw((adc2_channel_t)channel, width, &raw);
            adc_reading += raw;
        }
        adc_reading /= NO_OF_SAMPLES;

        u32 voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        angle       = mapv(adc_reading);
        printf("Raw: %ld\tVoltage: %ldmV\tAngle of rotation: %ld\n", adc_reading, voltage, angle);

        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angle_to_compare(angle)));

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
