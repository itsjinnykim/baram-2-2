#ifndef BARAM_ENCODER_H
#define BARAM_ENCODER_H
#include "def.h"
#include "motor_control.h"

/* ==================== encoder_recovery.h ==================== */
#ifndef ENCODER_RECOVERY_H
#define ENCODER_RECOVERY_H

#include <stdint.h>

typedef struct
{
    uint32_t first_cycles;
    uint32_t last_cycles;
    uint32_t samples;
} EncoderRecoveryGate;

/* The same C implementation is evaluated by compile-time C++ regression tests. */
#ifdef __cplusplus
#define ENCODER_RECOVERY_FN static constexpr
#else
#define ENCODER_RECOVERY_FN static inline
#endif

ENCODER_RECOVERY_FN uint32_t EncoderRecovery_Ready(
    EncoderRecoveryGate *gate, uint32_t now, uint32_t allow_rebase,
    uint32_t tracking_lost, uint32_t valid, uint32_t max_gap_cycles,
    uint32_t stable_cycles)
{
    if (!allow_rebase || !tracking_lost || !valid)
    {
        gate->samples = 0U;
        return 0U;
    }
    uint32_t gap = (uint32_t)(now - gate->last_cycles);
    if (!gate->samples || !gap || (gap > max_gap_cycles))
    {
        gate->first_cycles = now;
        gate->last_cycles = now;
        gate->samples = 1U;
        return 0U;
    }
    gate->last_cycles = now;
    gate->samples++;
    if ((gate->samples < 100U) ||
        ((uint32_t)(now - gate->first_cycles) < stable_cycles))
    {
        return 0U;
    }
    gate->samples = 0U;
    return 1U;
}

#undef ENCODER_RECOVERY_FN
#endif


/* ==================== encoder.h ==================== */
#ifndef ENCODER_H
#define ENCODER_H



#ifdef __cplusplus
extern "C" {
#endif
extern SPI_HandleTypeDef hspi3;
extern volatile EncoderDiagnostic g_encoder1;
extern volatile EncoderDiagnostic g_encoder2;
extern volatile Axis1MotorDiagnostic g_axis1_test;
extern volatile AxisPositionDiagnostic g_position;
typedef struct
{
    uint16_t cs_pin;
    uint32_t initialized;
    uint32_t last_cycles;
    uint32_t sample_cycles;
    EncoderRecoveryGate recovery;
    uint16_t last_raw;
    volatile EncoderDiagnostic *result;
} EncoderContext;
void MX_SPI3_Init(void);
void Encoder_TimebaseInit(void);
uint32_t Encoder_CyclesPerUs(void);
void Encoder_DeassertAll(void);
void Encoder_Poll(EncoderContext *encoder);
void Encoder_RecoverBeforeStart(EncoderContext *encoder,uint32_t allow_rebase);
#ifdef __cplusplus
}
#endif
#endif


#endif /* BARAM_ENCODER_H */
