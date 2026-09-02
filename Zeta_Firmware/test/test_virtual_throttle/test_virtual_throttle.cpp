#if __has_include(<unity.h>)
#include <unity.h>
#else
#include "../../.pio/libdeps/native/Unity/src/unity.h"
#endif

#include <stdint.h>
#include <stdio.h>

namespace
{
constexpr uint16_t PWM_PERIOD = 1000U;
constexpr uint16_t PWM_MIN = 40U;
constexpr uint16_t CURRENT_TRIP = 3800U;
constexpr uint16_t BEMF_TRIP = 3900U;
constexpr uint32_t LOOP_PERIOD_MS = 1U;

constexpr uint8_t COMMUTATION_STEPS = 6U;

constexpr uint8_t PHASE_TABLE[COMMUTATION_STEPS][2] = {{0, 1},
                                                       {0, 2},
                                                       {1, 2},
                                                       {1, 0},
                                                       {2, 0},
                                                       {2, 1}};

struct EscSnapshot
{
    uint32_t timeMs;
    uint16_t throttlePct;
    uint16_t duty;
    uint16_t currentSense;
    uint16_t bemfSense;
    uint8_t commStep;
    int highPhase;
    int lowPhase;
    uint16_t cmp0;
    uint16_t cmp1;
    uint16_t cmp2;
    bool l1Active;
    bool l2Active;
    bool l3Active;
    bool faulted;
};

class VirtualEsc
{
  public:
    EscSnapshot step(uint32_t timeMs, uint16_t throttlePct, uint16_t currentSense, uint16_t bemfSense)
    {
        const uint16_t clampedThrottle = throttlePct > 100U ? 100U : throttlePct;
        const uint16_t duty = static_cast<uint16_t>(
            static_cast<uint32_t>(clampedThrottle) * PWM_PERIOD / 100U);

        if (timeMs - lastStepMs_ > 4U)
        {
            if (clampedThrottle > 5U)
            {
                commStep_ = static_cast<uint8_t>((commStep_ + 1U) % COMMUTATION_STEPS);
            }
            else
            {
                commStep_ = 0U;
                allPhasesOff();
            }
            lastStepMs_ = timeMs;
        }

        if (clampedThrottle <= 5U)
        {
            allPhasesOff();
            lastDuty_ = 0U;
            return snapshot(timeMs, clampedThrottle, duty, currentSense, bemfSense, false);
        }

        if (duty > PWM_MIN && duty != lastDuty_)
        {
            applyCommutation(duty);
            lastDuty_ = duty;
        }

        const bool faulted = currentSense > CURRENT_TRIP || bemfSense > BEMF_TRIP;
        if (faulted)
        {
            allPhasesOff();
            commStep_ = 0U;
        }

        return snapshot(timeMs, clampedThrottle, duty, currentSense, bemfSense, faulted);
    }

  private:
    void applyCommutation(uint16_t duty)
    {
        const uint8_t high = PHASE_TABLE[commStep_][0];
        const uint8_t low = PHASE_TABLE[commStep_][1];

        allPhasesOff();

        switch (high)
        {
        case 0:
            cmp0_ = duty;
            break;
        case 1:
            cmp1_ = duty;
            break;
        case 2:
            cmp2_ = duty;
            break;
        default:
            break;
        }

        switch (low)
        {
        case 0:
            l1Active_ = true;
            break;
        case 1:
            l2Active_ = true;
            break;
        case 2:
            l3Active_ = true;
            break;
        default:
            break;
        }
    }

    void allPhasesOff()
    {
        cmp0_ = 0U;
        cmp1_ = 0U;
        cmp2_ = 0U;
        l1Active_ = false;
        l2Active_ = false;
        l3Active_ = false;
    }

    EscSnapshot snapshot(uint32_t timeMs,
                         uint16_t throttlePct,
                         uint16_t duty,
                         uint16_t currentSense,
                         uint16_t bemfSense,
                         bool faulted) const
    {
        int highPhase = -1;
        if (cmp0_ > 0U)
        {
            highPhase = 0;
        }
        else if (cmp1_ > 0U)
        {
            highPhase = 1;
        }
        else if (cmp2_ > 0U)
        {
            highPhase = 2;
        }

        int lowPhase = -1;
        if (l1Active_)
        {
            lowPhase = 0;
        }
        else if (l2Active_)
        {
            lowPhase = 1;
        }
        else if (l3Active_)
        {
            lowPhase = 2;
        }

        return {timeMs,
                throttlePct,
                duty,
                currentSense,
                bemfSense,
                commStep_,
                highPhase,
                lowPhase,
                cmp0_,
                cmp1_,
                cmp2_,
                l1Active_,
                l2Active_,
                l3Active_,
                faulted};
    }

    uint32_t lastStepMs_ = 0U;
    uint16_t lastDuty_ = 0U;
    uint8_t commStep_ = 0U;
    uint16_t cmp0_ = 0U;
    uint16_t cmp1_ = 0U;
    uint16_t cmp2_ = 0U;
    bool l1Active_ = false;
    bool l2Active_ = false;
    bool l3Active_ = false;
};

uint16_t simulatedCurrentSense(uint16_t throttlePct, uint32_t stepIndex)
{
    return static_cast<uint16_t>(220U + throttlePct * 24U + (stepIndex % 9U) * 7U);
}

uint16_t simulatedBemfSense(uint16_t throttlePct, uint32_t stepIndex)
{
    return static_cast<uint16_t>(140U + throttlePct * 32U + (stepIndex % 6U) * 11U);
}

void writeSnapshot(FILE *csv, const EscSnapshot &snapshot)
{
    fprintf(csv,
            "%u,%u,%u,%u,%u,%u,%d,%d,%u,%u,%u,%u,%u,%u,%u\n",
            snapshot.timeMs,
            snapshot.throttlePct,
            snapshot.duty,
            snapshot.currentSense,
            snapshot.bemfSense,
            snapshot.commStep,
            snapshot.highPhase,
            snapshot.lowPhase,
            snapshot.cmp0,
            snapshot.cmp1,
            snapshot.cmp2,
            snapshot.l1Active ? 1U : 0U,
            snapshot.l2Active ? 1U : 0U,
            snapshot.l3Active ? 1U : 0U,
            snapshot.faulted ? 1U : 0U);
}
} // namespace

void test_virtual_throttle_ramp_logs_all_steps()
{
    FILE *csv = fopen("test/zeta_data.csv", "w");
    TEST_ASSERT_NOT_NULL(csv);

    fprintf(csv,
            "time_ms,throttle_pct,duty,current_sense,bemf_sense,comm_step,"
            "high_phase,low_phase,cmp0,cmp1,cmp2,l1_active,l2_active,l3_active,faulted\n");

    VirtualEsc esc;
    bool sawDrivenPhase = false;
    bool sawFault = false;
    uint32_t rowCount = 0U;

    for (uint16_t throttlePct = 0U; throttlePct <= 100U; ++throttlePct)
    {
        const uint32_t stepIndex = throttlePct;
        const uint32_t timeMs = stepIndex * LOOP_PERIOD_MS;
        uint16_t currentSense = simulatedCurrentSense(throttlePct, stepIndex);
        uint16_t bemfSense = simulatedBemfSense(throttlePct, stepIndex);

        if (throttlePct == 100U)
        {
            currentSense = CURRENT_TRIP + 1U;
            bemfSense = 3600U;
        }

        const EscSnapshot snapshot = esc.step(timeMs, throttlePct, currentSense, bemfSense);
        writeSnapshot(csv, snapshot);
        ++rowCount;

        TEST_ASSERT_EQUAL_UINT16(throttlePct, snapshot.throttlePct);
        TEST_ASSERT_EQUAL_UINT16(throttlePct * 10U, snapshot.duty);

        if (throttlePct <= 5U || snapshot.faulted)
        {
            TEST_ASSERT_EQUAL_INT(-1, snapshot.highPhase);
            TEST_ASSERT_EQUAL_INT(-1, snapshot.lowPhase);
            TEST_ASSERT_EQUAL_UINT16(0U, snapshot.cmp0 + snapshot.cmp1 + snapshot.cmp2);
        }
        else if (snapshot.duty > PWM_MIN)
        {
            sawDrivenPhase = true;
            TEST_ASSERT_TRUE(snapshot.highPhase >= 0);
            TEST_ASSERT_TRUE(snapshot.lowPhase >= 0);
            TEST_ASSERT_NOT_EQUAL(snapshot.highPhase, snapshot.lowPhase);
        }

        if (snapshot.faulted)
        {
            sawFault = true;
            TEST_ASSERT_EQUAL_UINT8(0U, snapshot.commStep);
        }
    }

    TEST_ASSERT_EQUAL_UINT32(101U, rowCount);
    TEST_ASSERT_TRUE(sawDrivenPhase);
    TEST_ASSERT_TRUE(sawFault);

    fclose(csv);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_virtual_throttle_ramp_logs_all_steps);
    return UNITY_END();
}
