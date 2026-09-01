#include "utils.h"

float getDutyCycle(int pin)
{
    unsigned long high = pulseIn(pin, HIGH, PWM_TIMEOUT);
    unsigned long low = pulseIn(pin, LOW, PWM_TIMEOUT);

    unsigned long total = high + low;

    if (total == 0)
    {
        return 0;
    }
    else
    {
        int pwm_value = (high * 255) / total;

        float duty_cycle = ((float)high / total) * 100.0;

        return duty_cycle;
    }
}
