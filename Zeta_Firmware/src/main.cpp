#include <Arduino.h>
#include "utils.h"

#define H1 PIN_PB0
#define H2 PIN_PB1
#define H3 PIN_PB2
#define L1 PIN_PA3
#define L2 PIN_PA4
#define L3 PIN_PA5

#define CSNS_A PIN_PA7
#define CSNS_B PIN_PA6
#define THROTTLE_PIN PIN_PA1
#define PWM THROTTLE_PIN

#define PWM_PERIOD 1000U
#define PWM_MIN 40U
#define ADC_SAMPLES 4U

static const uint8_t PHASE_TABLE[6][2] = {
    {0, 1},
    {0, 2},
    {1, 2},
    {1, 0},
    {2, 0},
    {2, 1}
};

static uint8_t g_comm_step = 0;

static void set_high_side_pwm(uint8_t phase, uint16_t duty)
{
    uint16_t safeDuty = duty > PWM_PERIOD ? PWM_PERIOD : duty;

    switch (phase) {
        case 0:
            TCA0.SINGLE.CMP0 = safeDuty;
            break;
        case 1:
            TCA0.SINGLE.CMP1 = safeDuty;
            break;
        case 2:
            TCA0.SINGLE.CMP2 = safeDuty;
            break;
        default:
            break;
    }
}

static void set_low_side(uint8_t phase, bool active)
{
    uint8_t pin = 0;

    switch (phase) {
        case 0:
            pin = L1;
            break;
        case 1:
            pin = L2;
            break;
        case 2:
            pin = L3;
            break;
        default:
            return;
    }

    digitalWrite(pin, active ? LOW : HIGH);
}

static void all_phases_off(void)
{
    TCA0.SINGLE.CMP0 = 0;
    TCA0.SINGLE.CMP1 = 0;
    TCA0.SINGLE.CMP2 = 0;

    digitalWrite(L1, HIGH);
    digitalWrite(L2, HIGH);
    digitalWrite(L3, HIGH);
}

static void apply_commutation(uint16_t duty)
{
    const uint8_t high = PHASE_TABLE[g_comm_step][0];
    const uint8_t low  = PHASE_TABLE[g_comm_step][1];

    all_phases_off();
    set_high_side_pwm(high, duty);
    set_low_side(low, true);
}

static void timer_init(void)
{
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;
    TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_CMP1EN_bm | TCA_SINGLE_CMP2EN_bm;
    TCA0.SINGLE.CTRLC = 0;
    TCA0.SINGLE.CTRLD = 0;
    TCA0.SINGLE.PER = PWM_PERIOD;
    TCA0.SINGLE.CNT = 0;
    TCA0.SINGLE.CMP0 = 0;
    TCA0.SINGLE.CMP1 = 0;
    TCA0.SINGLE.CMP2 = 0;
}

static void adc_init(void)
{
    ADC0.CTRLC = (uint8_t)ADC_PRESC_DIV16_gc | (uint8_t)ADC_REFSEL_VDD_gc;
    ADC0.CTRLB = (uint8_t)ADC_MODE_SINGLE_12BIT_gc;
    ADC0.CTRLA = ADC_ENABLE_bm;
    ADC0.INTCTRL = 0;
    ADC0.INTFLAGS = ADC_RESRDY_bm;
    ADC0.MUXPOS = ADC_MUXPOS_AIN7_gc;
    ADC0.MUXNEG = ADC_MUXNEG_GND_gc;
}

static uint16_t adc_read_channel(uint8_t muxPos)
{
    uint32_t sum = 0;

    for (uint8_t i = 0; i < ADC_SAMPLES; ++i) {
        ADC0.MUXPOS = muxPos;
        ADC0.COMMAND = ADC_START_IMMEDIATE_gc;
        while (!(ADC0.INTFLAGS & ADC_RESRDY_bm)) {
            ;
        }
        ADC0.INTFLAGS = ADC_RESRDY_bm;
        sum += (uint16_t)ADC0.RESULT;
    }

    return (uint16_t)(sum / ADC_SAMPLES);
}

static uint16_t read_throttle_pwm_percent(void)
{
    const float duty = getDutyCycle(PWM);
    if (duty < 0.0f) {
        return 0;
    }
    if (duty > 100.0f) {
        return 100;
    }
    return (uint16_t)duty;
}

void setup()
{
    pinMode(H1, OUTPUT);
    pinMode(H2, OUTPUT);
    pinMode(H3, OUTPUT);
    pinMode(L1, OUTPUT);
    pinMode(L2, OUTPUT);
    pinMode(L3, OUTPUT);
    pinMode(PWM, INPUT);

    digitalWrite(H1, LOW);
    digitalWrite(H2, LOW);
    digitalWrite(H3, LOW);
    digitalWrite(L1, HIGH);
    digitalWrite(L2, HIGH);
    digitalWrite(L3, HIGH);

    timer_init();
    adc_init();
    all_phases_off();

    analogReadResolution(12);
    analogReference(VDD);
}

void loop()
{
    static uint32_t lastStepMs = 0;
    static uint16_t lastDuty = 0;

    uint16_t throttlePct = read_throttle_pwm_percent();
    uint16_t duty = (uint16_t)((uint32_t)throttlePct * PWM_PERIOD / 100U);

    const uint16_t currentSense = adc_read_channel(ADC_MUXPOS_AIN7_gc);
    const uint16_t bemfSense = adc_read_channel(ADC_MUXPOS_AIN6_gc);

    if (millis() - lastStepMs > 4) {
        if (throttlePct > 5U) {
            g_comm_step = (g_comm_step + 1U) % 6U;
        } else {
            g_comm_step = 0U;
            all_phases_off();
        }
        lastStepMs = millis();
    }

    if (throttlePct <= 5U) {
        all_phases_off();
        lastDuty = 0;
        return;
    }

    if (duty > PWM_MIN && duty != lastDuty) {
        apply_commutation(duty);
        lastDuty = duty;
    }

    if (currentSense > 3800U || bemfSense > 3900U) {
        all_phases_off();
        g_comm_step = 0U;
    }
}