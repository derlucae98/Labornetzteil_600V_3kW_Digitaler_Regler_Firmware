/**
 * @file    pwr_ctrl.h
 * @brief   Digital CV/CC controller for a hard-switched full bridge (STM32G474).
 *
 * Hardware chain per switching period, configured in CubeMX (.ioc):
 *
 *   HRTIM Timer E  CMP1 : TE1 + TE2 rise   -> both MCP33131 sample (CNVST)
 *                  CMP2 : TE1 falls        -> ADC_V drives the shared SDO
 *                         DMA req HRTIM1_E -> DMA writes pwr_spi_dummy to SPI1->DR
 *                         -> SPI1 clocks 16 bits -> SPI1_RX DMA -> raw[0]
 *                  CMP3 : TE2 falls        -> ADC_I drives SDO (ADC_V is Hi-Z)
 *                         DMA req again    -> second frame -> raw[1]
 *   SPI1_RX DMA transfer complete (2 words) -> control ISR (PWR_ADC_DMA_IRQHandler)
 *                         -> CV/CC 3p3z -> HRTIM Timer A compare registers
 *                         -> preload transferred at the next period start
 *
 * Hardware over-current trip, independent of the CPU:
 *   PC1 (I_SENSE) > DAC3 OUT1 -> COMP3 high -> HRTIM FLT5 -> TA1/TA2 forced low
 */
#ifndef PWR_CTRL_H
#define PWR_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal.h"
#include "pwr_config.h"

typedef enum
{
    PWR_STATE_OFF = 0,    /* bridge outputs disabled, ADCs keep sampling      */
    PWR_STATE_ARMED,      /* enable requested, executed inside the ISR        */
    PWR_STATE_RUN,        /* closed loop running                              */
    PWR_STATE_FAULT       /* tripped, call pwr_clear_fault() before restart   */
} pwr_state_t;

/* fault bits (latched) */
#define PWR_FAULT_NONE        0x00U
#define PWR_FAULT_OVP         0x01U
#define PWR_FAULT_OCP         0x02U
#define PWR_FAULT_CONFIG      0x04U   /* .ioc settings violate pwr_config.h limits */
#define PWR_FAULT_DMA         0x08U   /* DMA transfer error                        */
#define PWR_FAULT_WATCHDOG    0x10U   /* control ISR stopped running               */
#define PWR_FAULT_OCP_HW      0x20U   /* COMP3 tripped HRTIM FLT5 (PC1 > DAC3)     */

/* warning bits (informational, do not block the output) */
#define PWR_WARN_ADC_RECAL    0x01U   /* recalibration did not report completion   */
#define PWR_WARN_CNV_EDGE     0x02U   /* conversion overlaps the T/2 turn-on edge  */
#define PWR_WARN_SPI_TIMING   0x04U   /* rising-edge capture with SCK > 25 MHz     */

typedef struct
{
    /* live values */
    volatile float       v_meas;        /* [V] last output voltage sample      */
    volatile float       i_meas;        /* [A] last output current sample      */
    volatile float       v_ref;         /* [V] ramped voltage reference        */
    volatile float       duty;          /* applied duty (0 ... DUTY_MAX)       */
    volatile uint16_t    raw_v;         /* raw ADC codes                       */
    volatile uint16_t    raw_i;
    volatile uint8_t     cc_active;     /* 1 = current loop is limiting        */
    volatile pwr_state_t state;
    volatile uint32_t    fault;         /* PWR_FAULT_x                         */
    volatile uint32_t    warn;          /* PWR_WARN_x                          */
    volatile uint32_t    config_check;  /* number of the failed check, 0 = ok  */

    /* hardware over-current trip (DAC3 -> COMP3 -> HRTIM FLT5) */
    volatile float       ocp_hw_a;      /* [A] applied trip level (DAC-quantized) */
    volatile uint16_t    ocp_hw_dac;    /* DAC3 code, 12 bit                     */
    volatile uint8_t     ocp_hw_comp;   /* COMP3 output, 1 = above threshold     */
    volatile uint32_t    ocp_hw_trips;  /* control periods that saw a FLT5 event */

    /* ISR timing (HRTIM ticks, see t_tick_ps) */
    volatile uint32_t    isr_count;          /* increments every period        */
    volatile uint32_t    isr_exit_ticks;     /* Timer A counter at ISR exit    */
    volatile uint32_t    isr_exit_ticks_max; /* worst case                     */
    volatile uint32_t    isr_late_count;     /* ISR finished after period end  */

    /* timing read back from the CubeMX configuration */
    uint32_t             f_sw_hz;       /* switching frequency                 */
    uint32_t             t_tick_ps;     /* HRTIM tick (~184 ps)                */
    uint32_t             t_sample_ns;   /* CNVST rising (Timer E CMP1)         */
    uint32_t             t_read_v_ns;   /* CNVST_V falling (CMP2)              */
    uint32_t             t_read_i_ns;   /* CNVST_I falling (CMP3)              */
    uint32_t             spi_sck_hz;
} pwr_status_t;

/** DMA source for the SPI dummy write. Referenced by the .ioc
 *  (HRTIM1 -> Timer E -> DMA Src Address = (uint32_t)&pwr_spi_dummy). */
extern uint16_t pwr_spi_dummy;

/**
 * Arm the hardware OCP (DAC3 at PWR_OCP_HW_A, COMP3 on), check the CubeMX
 * configuration, recalibrate the ADCs, start the DMA chain and the HRTIM
 * counters. Afterwards both ADCs are sampled every period and the ISR runs,
 * but the bridge outputs stay disabled (state OFF).
 * Call once after the MX_xxx_Init() functions.
 */
void pwr_init(HRTIM_HandleTypeDef *hhrtim, SPI_HandleTypeDef *hspi,
              COMP_HandleTypeDef *hcomp, DAC_HandleTypeDef *hdac);

/** Set-points; clamped to PWR_VSET_MAX_V / PWR_ISET_MAX_A. Safe to call anytime. */
void pwr_set_voltage(float volts);
void pwr_set_current(float amps);

/** Request output enable. Returns false if a fault is latched. */
bool pwr_output_on(void);

/** Disable both bridge outputs immediately. */
void pwr_output_off(void);

/** Clear OVP/OCP faults (outputs stay off until pwr_output_on()).
 *  OCP_HW is cleared only if COMP3 is low again (current below the trip level).
 *  CONFIG, DMA and WATCHDOG faults need a reset. */
void pwr_clear_fault(void);

/** Hardware over-current trip level [A] (DAC3 threshold of COMP3). Takes effect
 *  after the DAC settling time, also while running. Clamped to the DAC range
 *  (0 ... PWR_DAC_VREF_V). Returns the level actually applied (12-bit DAC). */
float pwr_set_ocp_hw(float amps);

/** Live status (read-only). */
const pwr_status_t *pwr_status(void);

/** Call every 1 ms (SysTick_Handler): trips if the control ISR stops. */
void pwr_tick_1ms(void);

/** Register-only bridge shutdown for Error_Handler() / HardFault_Handler(). */
void pwr_emergency_off(void);

#ifdef __cplusplus
}
#endif

#endif /* PWR_CTRL_H */
