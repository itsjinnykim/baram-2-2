#ifndef DEF_H
#define DEF_H
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "motor_config.h"

/* 1161 operating limits, unchanged by source modularization. */
#define MOTOR_TEST_COMPARE 840U
#define ENCODER_POLL_US 1000U
#define ENCODER_MAX_GAP_US 5000U
#define ENCODER_RECOVERY_STABLE_US 200000U
typedef struct
{
    uint32_t polls;
    uint32_t frame_ok;
    uint32_t valid;
    uint32_t raw;
    float angle_deg;
    uint32_t diag;
    uint32_t magnitude;
    uint32_t loops_finished;
    uint32_t cordic_overflow;
    uint32_t mag_low;
    uint32_t mag_high;
    uint32_t good_reads;
    uint32_t spi_errors;
    uint32_t parity_errors;
    uint32_t device_errors;
    uint32_t errfl;
    uint32_t last_rx;
    uint32_t last_hal_error;
    uint32_t consecutive_good;
    uint32_t invalid_reads;
    uint32_t sample_interval_us;
    uint32_t tracking_valid; /* Relative position/speed usable; not homing. */
    uint32_t tracking_lost;  /* Latched after a gap/error; reset to start over. */
    uint32_t tracking_recoveries; /* Pre-start rebasings only; never during motion. */
    uint32_t last_tracking_fault; /* 1=invalid read, 2=gap, 3=zero dt, 4=half-turn. */
    uint32_t last_invalid_reason; /* Sticky: 1=SPI, 2=parity, 3=EF, 4=LF0,
                                    5=COF, 6=MAGL, 7=MAGH, 8=other. */
    uint32_t last_invalid_diag;   /* DIAAGC at the bad poll; UINT32_MAX if unread. */
    int64_t total_counts;   /* Relative to the first valid sample after reset. */
    float position_deg;    /* Unwrapped relative disk/output-shaft angle. */
    float speed_dps;       /* Filtered derivative, degrees/second. */
} EncoderDiagnostic;

typedef struct
{
    uint32_t valid;       /* ADC conversion success only; not current calibration. */
    uint32_t raw;         /* Rounded mean of 16 samples, with no zero subtraction. */
    uint32_t raw_min;     /* Min/max within the latest successful sample block. */
    uint32_t raw_max;
    float voltage_mv;    /* Approximate: uses g_adc_vdda_mv, initially 3300mV. */
    uint32_t samples;
    uint32_t errors;
    uint32_t fault_step;  /* 1=config, 2=start, 3=poll, 4=stop; 0=last block OK. */
    uint32_t hal_error;
} CurrentDiagnostic;

typedef struct
{
    uint32_t output_port; /* 2=TIM1/P2 R14/PC2; 4=TIM8/P4 R13/PC3. */
    uint32_t done;
    uint32_t valid;       /* Capture success, not calibrated current accuracy. */
    uint32_t fault;       /* Nonzero: setup/capture failed; see fault codes below. */
    uint32_t samples;
    uint32_t duration_ms;
    uint32_t sample_rate_hz;
    uint32_t trigger_rate_hz; /* Timer rate measured before drive (20k/40k). */
    float zero_mv;
    float mean_mv;
    float peak_mv;        /* Max sampled ON-window value, not all switching peaks. */
    float mean_a_est;     /* Nominal A1 gain 20 and 10mOhm; zero-subtracted estimate. */
    float peak_a_est;
    uint32_t split_ms;       /* Initial/later window boundary, currently 200ms. */
    uint32_t startup_samples;
    uint32_t steady_samples;
    float startup_mean_mv;   /* First 200ms, excluding the existing 8-sample guard. */
    float startup_peak_mv;
    float steady_mean_mv;    /* 200ms onward; does not assert mechanical settling. */
    float steady_peak_mv;
    uint32_t peak_at_ms_est; /* Approximate time of first largest sampled value. */
} MotorCurrentDiagnostic;

typedef struct
{
    uint32_t output_port; /* P2 only, TIM1. */
    uint32_t ready;       /* J1 feedback only; J2 is not required. */
    uint32_t done;
    uint32_t fault;       /* 1=encoder lost, 2=poll gap >5ms, 3=ADC/DMA lost. */
    uint32_t compare;
    uint32_t duration_ms;
    uint32_t run_samples;
    float start_deg;      /* Unwrapped relative position at drive start. */
    float end_deg;        /* Position at stop command; not a settling test. */
    float delta_deg;
    float peak_speed_dps; /* Absolute filtered speed during this test. */
} Axis1MotorDiagnostic;

typedef struct
{
    uint32_t axis;
    uint32_t output_port;
    uint32_t ready;
    uint32_t done;
    uint32_t reached;
    uint32_t fault; /* 1=encoder/gap, 2=4s timeout, 3=overshoot, 4=wrong direction. */
    uint32_t elapsed_ms;
    uint32_t settle_ms;
    uint32_t samples;
    uint32_t compare;
    uint32_t max_compare;
    float step_deg;
    float start_deg;
    float target_deg;
    float measured_deg;
    float error_deg;
    float delta_deg;
    float speed_ref_dps;
    float speed_dps;
    float final_error_deg;
} AxisPositionDiagnostic;
#endif
