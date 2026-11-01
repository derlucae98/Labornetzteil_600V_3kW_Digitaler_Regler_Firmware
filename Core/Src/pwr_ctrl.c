/**
 * @file    pwr_ctrl.c
 * @brief   CV/CC 3p3z control loop on top of the CubeMX-generated HRTIM/SPI/DMA
 *          configuration (STM32G474RETx, 2x MCP33131-10 on a shared SDO line).
 *
 * CubeMX (.ioc) configures:   this file adds:
 *   HRTIM1 Timer A (bridge)     - runtime check of the CubeMX timing
 *   HRTIM1 Timer E (ADC seq.)   - ADC recalibration via HRTIM software set/reset
 *   SPI1 (16 bit master)        - start of the DMA chain (HAL_DMA_Start*)
 *   DMA1 Ch1 (HRTIM1_E -> SPI)  - control ISR on the SPI1_RX DMA channel
 *   DMA1 Ch2 (SPI1_RX -> RAM)   - protection, watchdog, emergency off
 *   COMP3 (PC1 vs DAC3 OUT1)    - hardware OCP: threshold, arming, fault latch
 *   DAC3 OUT1 (internal)
 *   HRTIM1 FLT5 (from COMP3)
 *   pins, labels, NVIC
 *
 * Hard-switched full bridge: one output per diagonal.
 *   TA1 = diagonal 1 (Q1 + Q4): on from period start to CMP1     (1st half)
 *   TA2 = diagonal 2 (Q2 + Q3): on from CMP2 (= T/2) to CMP3     (2nd half)
 *
 * ADC sampling position (Timer E CMP1..3), chosen per period with the duty:
 *   low duty : off-time after diagonal 1 turns off  (CubeMX compare values)
 *   high duty: on-time of diagonal 2, PWR_SAMPLE_ON_SETTLE_NS after it turns on
 *              (same compare spacing, shifted; computed in pwr_init())
 *   Switch with hysteresis (PWR_SAMPLE_HI_ENTER/EXIT_PERMILLE). The PWM itself
 *   is not affected; both positions are valid in the hysteresis band.
 *
 * Hardware OCP: PC1 > DAC3 -> COMP3 -> FLT5 forces TA1/TA2 low without the
 * CPU. The control ISR only latches the event (PWR_FAULT_OCP_HW).
 *
 * Pin labels (main.h): DIAG1_Q1_Q4 PA8, DIAG2_Q2_Q3 PA9, ADC_V_CNVST PC8,
 * ADC_I_CNVST PC9, ADC_SCLK PA5, ADC_SDO PA6, I_SENSE PC1, DBG_ISR PB1.
 */

#pragma GCC optimize ("O2")   /* keep the ISR fast even in a -O0 Debug build */

#include "pwr_ctrl.h"
#include "comp3p3z.h"
#include "main.h"

/* Optional: place the ISR in SRAM, e.g. -DPWR_RAMFUNC='__attribute__((section(".RamFunc")))' */
#ifndef PWR_RAMFUNC
#define PWR_RAMFUNC
#endif

/* -------------------------------------------------------------------------- */
/* Hardware shortcuts (HRTIM1 is the only HRTIM instance on the G474)         */
/* -------------------------------------------------------------------------- */
#define TIM_A   (HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A])   /* bridge, TA1/TA2 */
#define TIM_E   (HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E])   /* ADC sequencer  */

#define BRIDGE_OEN   (HRTIM_OENR_TA1OEN | HRTIM_OENR_TA2OEN)
#define BRIDGE_ODIS  (HRTIM_ODISR_TA1ODIS | HRTIM_ODISR_TA2ODIS)

#define HRTIM_CMP_MIN   0x60U          /* smallest valid compare value (CKPSC=0) */
#define PWR_DUTY_MAX    ((float)PWR_DUTY_MAX_PERMILLE / 1000.0f)

#if PWR_DEBUG_PIN_ENABLE
#define DBG_HIGH()   (DBG_ISR_GPIO_Port->BSRR = (uint32_t)DBG_ISR_Pin)
#define DBG_LOW()    (DBG_ISR_GPIO_Port->BRR  = (uint32_t)DBG_ISR_Pin)
#else
#define DBG_HIGH()   ((void)0)
#define DBG_LOW()    ((void)0)
#endif

/* -------------------------------------------------------------------------- */
/* State                                                                      */
/* -------------------------------------------------------------------------- */
uint16_t pwr_spi_dummy = 0xFFFFU;               /* HRTIM DMA source (see .ioc)   */

static volatile uint16_t s_adc_raw[2];           /* [0] = V, [1] = I, from DMA    */

static HRTIM_HandleTypeDef *s_hrtim;
static SPI_HandleTypeDef   *s_spi;
static COMP_HandleTypeDef  *s_comp;              /* COMP3: hardware OCP           */
static DAC_HandleTypeDef   *s_dac;               /* DAC3 OUT1: OCP threshold      */
static DMA_TypeDef         *s_rx_dma;            /* SPI1_RX DMA controller        */
static uint32_t             s_rx_flag_shift;     /* channel index in ISR/IFCR     */

static comp3p3z_t   s_cv;                        /* voltage loop                  */
static comp3p3z_t   s_cc;                        /* current loop                  */
static pwr_status_t s_st;
static volatile float s_vset;                    /* target voltage [V]           */
static volatile float s_iset;                    /* current limit [A]            */
static float          s_vref;                    /* ramped reference (ISR only)  */
static float          s_vref_step;               /* [V] per switching period     */
static volatile bool  s_running;                 /* DMA chain + ISR active        */

/* timing in HRTIM ticks, read from the CubeMX configuration */
static uint32_t s_half;                          /* T/2: Q2/Q3 turn-on            */
static float    s_half_f;
static uint32_t s_ton_min;
static uint32_t s_ton_max;

/* ADC sampling position: [0] = off-time (CubeMX), [1] = on-time of diagonal 2 */
typedef struct { uint32_t cmp1, cmp2, cmp3; } te_cmp_t;
static te_cmp_t s_te[2];
static uint32_t s_ton_hi_enter;                  /* on-time above: use [1]        */
static uint32_t s_ton_hi_exit;                   /* on-time below: back to [0]    */
static uint32_t s_pos_next;                      /* position written for next period */
static uint32_t s_pos_cur;                       /* position of this period's sample */

/* -------------------------------------------------------------------------- */
/* Helpers                                                                    */
/* -------------------------------------------------------------------------- */
static uint64_t s_fhr;                           /* HRTIM tick frequency [Hz]     */

static uint32_t ns_to_ticks(uint32_t ns)
{
    return (uint32_t)(((uint64_t)ns * s_fhr + 500000000ULL) / 1000000000ULL);
}

static uint32_t ticks_to_ns(uint32_t t)
{
    return (uint32_t)(((uint64_t)t * 1000000000ULL) / s_fhr);
}

__attribute__((always_inline)) static inline void bridge_off(void)
{
    HRTIM1->sCommonRegs.ODISR = BRIDGE_ODIS;
}

/* interrupt lock for state changes outside the control ISR */
#define CRITICAL_ENTER()  const uint32_t primask_ = __get_PRIMASK(); __disable_irq()
#define CRITICAL_EXIT()   __set_PRIMASK(primask_)

static void trip(uint32_t fault)
{
    CRITICAL_ENTER();
    bridge_off();
    s_st.fault |= fault;
    s_st.state  = PWR_STATE_FAULT;
    CRITICAL_EXIT();
}

/* COMP3 output: true while PC1 is above the DAC3 threshold (FLT5 active) */
__attribute__((always_inline)) static inline bool ocp_hw_active(void)
{
    return (s_comp != NULL) && ((s_comp->Instance->CSR & COMP_CSR_VALUE) != 0U);
}

/* Write the duty of both diagonals and the ADC sampling position for the same
 * period. Timer A and E roll over together; update disable makes the group
 * atomic, so duty and sampling position always change at the same period start
 * (or, if the write straddles it, together one period later). */
__attribute__((always_inline)) static inline void set_duty(float d)
{
    if (!(d >= 0.0f))       { d = 0.0f; }
    else if (d > PWR_DUTY_MAX) { d = PWR_DUTY_MAX; }

    uint32_t ton = (uint32_t)(d * s_half_f);
    if (ton < s_ton_min) { ton = s_ton_min; }
    if (ton > s_ton_max) { ton = s_ton_max; }

    uint32_t pos = s_pos_next;                   /* hysteresis on the on-time     */
    if (ton > s_ton_hi_enter)     { pos = 1U; }
    else if (ton < s_ton_hi_exit) { pos = 0U; }

    HRTIM1->sCommonRegs.CR1 |= (HRTIM_CR1_TAUDIS | HRTIM_CR1_TEUDIS);
    TIM_A.CMP1xR = ton;
    TIM_A.CMP3xR = s_half + ton;
    if (pos != s_pos_next)
    {
        TIM_E.CMP1xR = s_te[pos].cmp1;
        TIM_E.CMP2xR = s_te[pos].cmp2;
        TIM_E.CMP3xR = s_te[pos].cmp3;
        s_pos_next = pos;
        s_st.sample_switches++;
    }
    HRTIM1->sCommonRegs.CR1 &= ~(HRTIM_CR1_TAUDIS | HRTIM_CR1_TEUDIS);
}

/* -------------------------------------------------------------------------- */
/* Configuration check: the .ioc must match the assumptions of this file      */
/* Returns 0 if ok, otherwise the number of the failed check (debugger).      */
/* -------------------------------------------------------------------------- */
static uint32_t check_dma_config(void)
{
    const HRTIM_TimerParamTypeDef *te = &s_hrtim->TimerParam[HRTIM_TIMERINDEX_TIMER_E];

    if (s_hrtim->Instance != HRTIM1)                                    { return 1U; }
    if ((s_spi->hdmarx == NULL) || (s_spi->hdmarx->Instance != PWR_ADC_DMA_CHANNEL))
                                                                        { return 2U; }
    if (s_spi->hdmarx->Init.Direction != DMA_PERIPH_TO_MEMORY)          { return 3U; }
    if (s_spi->hdmarx->Init.Mode != DMA_CIRCULAR)                       { return 4U; }
    if (s_spi->hdmarx->Init.PeriphDataAlignment != DMA_PDATAALIGN_HALFWORD) { return 5U; }
    if (s_spi->hdmarx->Init.MemDataAlignment != DMA_MDATAALIGN_HALFWORD)    { return 6U; }
    if (s_spi->hdmarx->Init.MemInc != DMA_MINC_ENABLE)                  { return 7U; }
    if (s_hrtim->hdmaTimerE == NULL)                                    { return 8U; }
    if (s_hrtim->hdmaTimerE->Init.Direction != DMA_MEMORY_TO_PERIPH)    { return 9U; }
    if (s_hrtim->hdmaTimerE->Init.Mode != DMA_CIRCULAR)                 { return 10U; }
    if (s_hrtim->hdmaTimerE->Init.PeriphDataAlignment != DMA_PDATAALIGN_HALFWORD) { return 11U; }
    if (te->DMARequests != (HRTIM_TIM_DMA_CMP2 | HRTIM_TIM_DMA_CMP3))   { return 12U; }
    if (te->DMASrcAddress != (uint32_t)&pwr_spi_dummy)                  { return 13U; }
    if (te->DMADstAddress != (uint32_t)&s_spi->Instance->DR)            { return 14U; }
    if (te->DMASize != 1U)                                              { return 15U; }
    if (s_spi->Init.Mode != SPI_MODE_MASTER)                            { return 16U; }
    if (s_spi->Init.Direction != SPI_DIRECTION_2LINES)                  { return 17U; }
    if (s_spi->Init.DataSize != SPI_DATASIZE_16BIT)                     { return 18U; }
    if (s_spi->Init.CLKPolarity != SPI_POLARITY_LOW)                    { return 19U; }
    if (s_spi->Init.NSS != SPI_NSS_SOFT)                                { return 20U; }
    if (s_spi->Init.FirstBit != SPI_FIRSTBIT_MSB)                       { return 21U; }
    return 0U;
}

static uint32_t check_timing_config(void)
{
    const uint32_t per = TIM_A.PERxR;

    /* prescaler x32 (CKPSC = 0) is assumed by the compare-value limits */
    if ((TIM_A.TIMxCR & HRTIM_TIMCR_CK_PSC) != 0U)                      { return 30U; }
    if ((TIM_E.TIMxCR & HRTIM_TIMCR_CK_PSC) != 0U)                      { return 31U; }
    if (TIM_E.PERxR != per)                                             { return 32U; }
    if (TIM_A.CMP2xR != s_half)                                         { return 33U; }
    if ((TIM_A.TIMxCR & HRTIM_TIMCR_PREEN) == 0U)                       { return 34U; }
    if ((TIM_A.TIMxCR & HRTIM_TIMCR_TRSTU) == 0U)                       { return 35U; }
    /* diagonal 1 only in the first half, diagonal 2 only in the second half:
     * any other set/reset source could turn on both diagonals (shoot-through) */
    if ((TIM_A.SETx1R != HRTIM_OUTPUTSET_TIMPER)   || (TIM_A.RSTx1R != HRTIM_OUTPUTRESET_TIMCMP1) ||
        (TIM_A.SETx2R != HRTIM_OUTPUTSET_TIMCMP2)  || (TIM_A.RSTx2R != HRTIM_OUTPUTRESET_TIMCMP3))
                                                                        { return 36U; }
    /* active high, no dead-time unit (it would make TA2 the complement of TA1) */
    if ((TIM_A.OUTxR & (HRTIM_OUTR_POL1 | HRTIM_OUTR_POL2 | HRTIM_OUTR_DTEN)) != 0U)
                                                                        { return 37U; }

    const uint32_t sample = TIM_E.CMP1xR;
    const uint32_t read_v = TIM_E.CMP2xR;
    const uint32_t read_i = TIM_E.CMP3xR;
    const uint32_t frame  = ns_to_ticks((uint32_t)((16ULL * 1000000000ULL
                                         + s_st.spi_sck_hz - 1U) / s_st.spi_sck_hz));

    const uint32_t settle_off = ns_to_ticks(PWR_SAMPLE_SETTLE_NS);
    const uint32_t settle_on  = ns_to_ticks(PWR_SAMPLE_ON_SETTLE_NS);
    const uint32_t tcnv       = ns_to_ticks(PWR_ADC_TCNV_NS);
    const uint32_t budget     = ns_to_ticks(PWR_COMPUTE_BUDGET_NS);
    const uint32_t ton_enter  = (uint32_t)(((uint64_t)s_half * PWR_SAMPLE_HI_ENTER_PERMILLE) / 1000U);
    const uint32_t ton_exit   = (uint32_t)(((uint64_t)s_half * PWR_SAMPLE_HI_EXIT_PERMILLE) / 1000U);

    /* off-time position = CubeMX values; used alone until all checks passed */
    s_te[0].cmp1 = sample;
    s_te[0].cmp2 = read_v;
    s_te[0].cmp3 = read_i;
    s_te[1] = s_te[0];

    if (s_ton_max <= s_ton_min)                                         { return 40U; }
    if (read_v < sample + tcnv)                                         { return 42U; }
    if (read_i < read_v + frame + ns_to_ticks(PWR_ADC_DMA_MARGIN_NS))   { return 43U; }
    if (read_i + frame + budget > per)                                  { return 44U; }
    if (s_half + s_ton_max >= per)                                      { return 45U; }
    /* shoot-through margin: diagonal 1 off -> diagonal 2 on (same legs) */
    if (s_half - s_ton_max < ns_to_ticks(PWR_DIAG_GAP_MIN_NS))          { return 48U; }

    /* --- sampling positions ----------------------------------------------- */
    if (sample < settle_off)                                            { return 41U; }
    const uint32_t ton_off_max = sample - settle_off; /* off-time position valid up to here */

    if (s_ton_max > ton_off_max)
    {
        /* on-time of diagonal 2: same spacing as the CubeMX set, shifted */
        if (sample > s_half + settle_on)                                { return 46U; }
        const uint32_t shift = (s_half + settle_on) - sample;
        const te_cmp_t hi = { sample + shift, read_v + shift, read_i + shift };
        /* off-time position clear of the turn-off edge up to the switch-over */
        if (ton_enter > ton_off_max)                                    { return 41U; }
        /* on-time position: conversion done before diagonal 2 turns off, down
         * to the switch-back point */
        if (hi.cmp1 + tcnv > s_half + ton_exit)                         { return 46U; }
        /* ISR keeps its compute budget before the period ends */
        if (hi.cmp3 + frame + budget > per)                             { return 47U; }

        s_te[1]        = hi;
        s_ton_hi_enter = ton_enter;
        s_ton_hi_exit  = ton_exit;
    }
    /* else: the off-time position covers the whole duty range, never switch */

    if (sample + ns_to_ticks(750U) > s_half)       { s_st.warn |= PWR_WARN_CNV_EDGE; }
    if ((s_spi->Init.CLKPhase == SPI_PHASE_1EDGE) && (s_st.spi_sck_hz > 25000000U))
                                                   { s_st.warn |= PWR_WARN_SPI_TIMING; }

    s_st.t_sample_ns    = ticks_to_ns(sample);
    s_st.t_read_v_ns    = ticks_to_ns(read_v);
    s_st.t_read_i_ns    = ticks_to_ns(read_i);
    s_st.t_sample_hi_ns = ticks_to_ns(s_te[1].cmp1);
    return 0U;
}

/* Hardware OCP path: PC1 -> COMP3(+), DAC3 OUT1 -> COMP3(-), COMP3 -> FLT5 -> TA1/TA2 */
static uint32_t check_ocp_config(void)
{
    const uint32_t fltinr2 = HRTIM1->sCommonRegs.FLTINR2;
    const uint32_t fltinr4 = HRTIM1->sCommonRegs.FLTINR4;

    /* FLT5's internal source is COMP3 (RM0440, HRTIM fault inputs) */
    if ((s_comp == NULL) || (s_comp->Instance != COMP3))                { return 50U; }
    if (s_comp->Init.InputMinus != COMP_INPUT_MINUS_DAC3_CH1)          { return 51U; }
    if (s_comp->Init.InputPlus != COMP_INPUT_PLUS_IO2)                 { return 52U; } /* PC1 */
    if (s_comp->Init.OutputPol != COMP_OUTPUTPOL_NONINVERTED)          { return 53U; }
    if (s_comp->Init.BlankingSrce != COMP_BLANKINGSRC_NONE)            { return 54U; }
    if ((s_dac == NULL) || (s_dac->Instance != DAC3))                   { return 55U; }
    /* FLT5 enabled, active high, internal source (SRC = 01) */
    if ((fltinr2 & (HRTIM_FLTINR2_FLT5E | HRTIM_FLTINR2_FLT5P | HRTIM_FLTINR2_FLT5SRC_0 |
                    HRTIM_FLTINR2_FLT5SRC_1)) !=
        (HRTIM_FLTINR2_FLT5E | HRTIM_FLTINR2_FLT5P | HRTIM_FLTINR2_FLT5SRC_0))
                                                                        { return 56U; }
    /* no blanking window, no fault counter: every event trips */
    if ((fltinr4 & (HRTIM_FLTINR4_FLT5BLKE | HRTIM_FLTINR4_FLT5CNT)) != 0U) { return 57U; }
    /* Timer A (bridge) reacts to FLT5 and drives both outputs inactive */
    if ((TIM_A.FLTxR & HRTIM_FLTR_FLT5EN) == 0U)                        { return 58U; }
    if ((TIM_A.OUTxR & (HRTIM_OUTR_FAULT1 | HRTIM_OUTR_FAULT2)) !=
        (HRTIM_OUTR_FAULT1_1 | HRTIM_OUTR_FAULT2_1))                    { return 59U; }
    /* scaling from pwr_config.h: the start-up level must fit the DAC range */
    if (!(PWR_OCP_HW_SENSE_V_PER_A > 0.0f))                             { return 60U; }
    const float v_trip = PWR_OCP_HW_A * PWR_OCP_HW_SENSE_V_PER_A + PWR_OCP_HW_SENSE_OFFSET_V;
    if (!(v_trip > 0.0f) || !(v_trip < PWR_DAC_VREF_V))                 { return 61U; }
    return 0U;
}

#if PWR_ADC_RECAL_AT_INIT
/* CNVST levels through the HRTIM software set/reset (counters stopped) */
static void cnvst(uint32_t output, bool high)
{
    (void)HAL_HRTIM_WaveformSetOutputLevel(s_hrtim, HRTIM_TIMERINDEX_TIMER_E, output,
                                           high ? HRTIM_OUTPUTLEVEL_ACTIVE
                                                : HRTIM_OUTPUTLEVEL_INACTIVE);
}

/* MCP33131 recalibration of ONE ADC: a conversion, then CNVST low + 1024 SCLK
 * pulses. The ADC pulls SDO low while calibrating and releases it when done;
 * the SDO pull-up then reads high. The other ADC keeps CNVST high, so it
 * ignores the clocks and its SDO stays Hi-Z (no contention on the shared line). */
static bool adc_recalibrate_one(uint32_t sel, uint32_t other)
{
    uint16_t tx[16];
    uint16_t rx[16];
    for (uint32_t k = 0U; k < 16U; k++) { tx[k] = 0xFFFFU; }

    cnvst(other, true);                                      /* deselect       */
    cnvst(sel, true);                                        /* conversion     */
    HAL_Delay(1U);                                           /* >> tCNV        */
    cnvst(sel, false);                                       /* select         */

    for (uint32_t k = 0U; k < 4U; k++)                       /* 4 x 16 x 16 = 1024 */
    {
        if (HAL_SPI_TransmitReceive(s_spi, (uint8_t *)tx, (uint8_t *)rx, 16U, 10U) != HAL_OK)
        {
            return false;
        }
    }

    HAL_Delay(2U);
    const uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < PWR_ADC_RECAL_TIMEOUT_MS)
    {
        if (HAL_GPIO_ReadPin(ADC_SDO_GPIO_Port, ADC_SDO_Pin) == GPIO_PIN_SET)
        {
            HAL_Delay(1U);                                   /* debounce       */
            if (HAL_GPIO_ReadPin(ADC_SDO_GPIO_Port, ADC_SDO_Pin) == GPIO_PIN_SET)
            {
                return true;
            }
        }
    }
    return false;
}

static bool adc_recalibrate(void)
{
    bool ok = adc_recalibrate_one(HRTIM_OUTPUT_TE1, HRTIM_OUTPUT_TE2);    /* ADC_V */
    ok = adc_recalibrate_one(HRTIM_OUTPUT_TE2, HRTIM_OUTPUT_TE1) && ok;   /* ADC_I */
    /* ADC_I is calibrated and Hi-Z; ADC_V holds the conversion started above
     * and drives SDO alone until the first HRTIM CMP1 raises both CNVST. */
    cnvst(HRTIM_OUTPUT_TE1, false);
    return ok;
}
#endif

/* HRTIM Timer E -> SPI DMA error (generated DMA1_Channel1_IRQHandler -> HAL) */
static void hrtim_dma_error(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    s_running = false;
    trip(PWR_FAULT_DMA);
}

/* -------------------------------------------------------------------------- */
/* Control ISR: once per period, as soon as both ADC words are in RAM         */
/* (~1.9 us after the sampling instant with the default .ioc timing)          */
/* -------------------------------------------------------------------------- */
PWR_RAMFUNC void PWR_ADC_DMA_IRQHandler(void)
{
    const uint32_t isr = s_rx_dma->ISR >> s_rx_flag_shift;
    s_rx_dma->IFCR = DMA_FLAG_GL1 << s_rx_flag_shift;

    if ((isr & DMA_FLAG_TE1) != 0U)          /* channel was disabled by hardware */
    {
        s_running = false;
        trip(PWR_FAULT_DMA);
        return;
    }
    if ((isr & DMA_FLAG_TC1) == 0U)
    {
        return;
    }
    DBG_HIGH();

    const uint16_t rv = s_adc_raw[0];
    const uint16_t ri = s_adc_raw[1];
    const float v = (float)rv * (PWR_V_FULLSCALE_V / 65536.0f) + PWR_V_OFFSET_V;
    const float i = (float)ri * (PWR_I_FULLSCALE_A / 65536.0f) + PWR_I_OFFSET_A;

    pwr_state_t st = s_st.state;
    float d = 0.0f;

    /* hardware OCP: FLT5 has already forced TA1/TA2 low (asynchronously, without
     * the CPU). Latch it here, in every state, so that the bridge stays off. */
    if ((HRTIM1->sCommonRegs.ISR & HRTIM_ISR_FLT5) != 0U)
    {
        HRTIM1->sCommonRegs.ICR = HRTIM_ICR_FLT5C;
        bridge_off();                            /* TA1/TA2: FAULT -> IDLE state  */
        s_st.fault |= PWR_FAULT_OCP_HW;
        s_st.ocp_hw_trips++;
        st = PWR_STATE_FAULT;
    }

    /* software protection on the ADC readings */
    if ((st == PWR_STATE_RUN) || (st == PWR_STATE_ARMED))
    {
        uint32_t f = 0U;
        if (v > PWR_OVP_V) { f |= PWR_FAULT_OVP; }
        if (i > PWR_OCP_A) { f |= PWR_FAULT_OCP; }
        if (f != 0U)
        {
            bridge_off();
            s_st.fault |= f;
            st = PWR_STATE_FAULT;
        }
    }

    if ((st == PWR_STATE_ARMED) && (s_st.fault != PWR_FAULT_NONE))
    {
        st = PWR_STATE_FAULT;                    /* never enable over a latched fault */
    }

    if (st == PWR_STATE_ARMED)
    {
        /* The duty has been at its minimum and we are past both turn-off edges,
         * so enabling the outputs here cannot create a truncated first pulse. */
        comp3p3z_reset(&s_cv, 0.0f);
        comp3p3z_reset(&s_cc, 0.0f);
        s_vref = (v > 0.0f) ? v : 0.0f;          /* ramp starts at present Vout */
        set_duty(0.0f);
        HRTIM1->sCommonRegs.OENR = BRIDGE_OEN;
        st = PWR_STATE_RUN;
    }
    else if (st == PWR_STATE_RUN)
    {
        /* reference ramp (soft start) */
        const float vset = s_vset;
        if (!(s_vref_step > 0.0f))              { s_vref = vset; }
        else if (s_vref < vset - s_vref_step)   { s_vref += s_vref_step; }
        else if (s_vref > vset + s_vref_step)   { s_vref -= s_vref_step; }
        else                                    { s_vref = vset; }

#if PWR_OPEN_LOOP
        d = (float)PWR_OPEN_LOOP_PERMILLE / 1000.0f;
        set_duty(d);
        s_st.cc_active = 0U;
#else
        const float uv = comp3p3z_calc(&s_cv, s_vref - v);
        const float ui = comp3p3z_calc(&s_cc, s_iset - i);
        d = (ui < uv) ? ui : uv;                 /* CV/CC: lower duty wins       */
        set_duty(d);                             /* <- latency-critical part ends */
        comp3p3z_commit(&s_cv, d);               /* both loops track the applied  */
        comp3p3z_commit(&s_cc, d);               /* duty = anti-windup            */
        s_st.cc_active = (ui < uv) ? 1U : 0U;
#endif
    }
    else
    {
        set_duty(0.0f);                          /* OFF / FAULT: minimum pulses   */
    }

    s_st.raw_v  = rv;
    s_st.raw_i  = ri;
    s_st.v_meas = v;
    s_st.i_meas = i;
    s_st.v_ref  = s_vref;
    s_st.duty   = d;
    s_st.state  = st;
    s_st.ocp_hw_comp = ocp_hw_active() ? 1U : 0U;
    s_st.sample_hi   = (uint8_t)s_pos_cur;
    s_st.isr_count++;

    const uint32_t cnt = TIM_A.CNTxR;
    s_st.isr_exit_ticks = cnt;
    if (cnt > s_st.isr_exit_ticks_max) { s_st.isr_exit_ticks_max = cnt; }
    if (cnt < s_te[s_pos_cur].cmp3) { s_st.isr_late_count++; }  /* counter wrapped */
    s_pos_cur = s_pos_next;                      /* active from the next period   */

    DBG_LOW();
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */
void pwr_init(HRTIM_HandleTypeDef *hhrtim, SPI_HandleTypeDef *hspi,
              COMP_HandleTypeDef *hcomp, DAC_HandleTypeDef *hdac)
{
    s_hrtim = hhrtim;
    s_spi   = hspi;
    s_comp  = hcomp;
    s_dac   = hdac;
    s_running = false;
    s_st.state = PWR_STATE_OFF;
    s_st.fault = PWR_FAULT_NONE;
    s_st.warn  = 0U;
    s_vset = 0.0f;
    s_iset = 0.0f;
    s_vref = 0.0f;

    bridge_off();

    comp3p3z_init(&s_cv, PWR_CV_B0, PWR_CV_B1, PWR_CV_B2, PWR_CV_B3,
                  PWR_CV_A1, PWR_CV_A2, PWR_CV_A3, 0.0f, PWR_DUTY_MAX);
    comp3p3z_init(&s_cc, PWR_CC_B0, PWR_CC_B1, PWR_CC_B2, PWR_CC_B3,
                  PWR_CC_A1, PWR_CC_A2, PWR_CC_A3, 0.0f, PWR_DUTY_MAX);

    /* --- timing from the CubeMX configuration ---------------------------- */
    /* HRTIM kernel clock = SYSCLK on the G4 (CubeMX: HRTIM1Freq = SYSCLK) */
    s_fhr = (uint64_t)HAL_RCC_GetSysClockFreq() * 32U;
    const uint32_t per = TIM_A.PERxR;
    s_half   = per / 2U;
    s_half_f = (float)s_half;
    s_ton_min = ns_to_ticks(PWR_TON_MIN_NS);
    if (s_ton_min < HRTIM_CMP_MIN) { s_ton_min = HRTIM_CMP_MIN; }
    s_ton_max = (uint32_t)(((uint64_t)s_half * PWR_DUTY_MAX_PERMILLE) / 1000U);
    s_ton_hi_enter = 0xFFFFFFFFU;                /* no switching until checked    */
    s_ton_hi_exit  = 0U;
    s_pos_next = 0U;                             /* start with the CubeMX values  */
    s_pos_cur  = 0U;

    s_st.t_tick_ps  = (uint32_t)(1000000000000ULL / s_fhr);
    s_st.f_sw_hz    = (uint32_t)(s_fhr / per);
    s_st.spi_sck_hz = HAL_RCC_GetPCLK2Freq() >> ((s_spi->Init.BaudRatePrescaler >> SPI_CR1_BR_Pos) + 1U);
    s_vref_step     = PWR_VREF_SLEW_V_PER_S / (float)s_st.f_sw_hz;

    /* --- hardware OCP first: threshold, comparator, FLT5 lock ------------- */
    s_st.config_check = check_ocp_config();
    if (s_st.config_check != 0U)
    {
        s_st.fault |= PWR_FAULT_CONFIG;          /* bridge stays locked           */
        s_st.state  = PWR_STATE_FAULT;
    }
    if ((s_dac != NULL) && (s_dac->Instance == DAC3) &&
        (s_comp != NULL) && (s_comp->Instance == COMP3))
    {
        if (HAL_DAC_Start(s_dac, DAC_CHANNEL_1) != HAL_OK) { Error_Handler(); }
        (void)pwr_set_ocp_hw(PWR_OCP_HW_A);
        if (HAL_COMP_Start(s_comp) != HAL_OK)              { Error_Handler(); }
        if (HAL_COMP_Lock(s_comp) != HAL_OK)               { Error_Handler(); }
        HAL_Delay(1U);                           /* DAC settling, COMP start-up   */
        HRTIM1->sCommonRegs.ICR = HRTIM_ICR_FLT5C;   /* discard start-up events   */
        bridge_off();                            /* TA1/TA2 in IDLE, not FAULT    */
        if (s_st.config_check == 0U)
        {
            /* FLT5 configuration read-only until reset (enable bit included) */
            HRTIM1->sCommonRegs.FLTINR2 |= HRTIM_FLTINR2_FLT5LCK;
        }
        if (ocp_hw_active())                     /* already above the threshold   */
        {
            s_st.fault |= PWR_FAULT_OCP_HW;
            s_st.state  = PWR_STATE_FAULT;
        }
    }
    if (s_st.config_check != 0U)
    {
        return;                                  /* nothing else is started       */
    }

    s_st.config_check = check_dma_config();
    if (s_st.config_check != 0U)
    {
        /* never start a DMA with unexpected settings: the chain stays off */
        s_st.fault |= PWR_FAULT_CONFIG;
        s_st.state  = PWR_STATE_FAULT;
        return;
    }
    s_st.config_check = check_timing_config();
    if (s_st.config_check != 0U)
    {
        s_st.fault |= PWR_FAULT_CONFIG;          /* ADCs run, bridge stays locked */
    }

    /* --- ADC recalibration (CNVST via HRTIM, counters still stopped) ------ */
    if (HAL_HRTIM_WaveformOutputStart(s_hrtim, HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2) != HAL_OK)
    {
        Error_Handler();
    }
#if PWR_ADC_RECAL_AT_INIT
    HAL_Delay(PWR_ADC_RECAL_DELAY_MS);
    if (!adc_recalibrate()) { s_st.warn |= PWR_WARN_ADC_RECAL; }
#endif

    /* --- SPI1 RX DMA: 2 half-words per period, circular, control ISR ------ */
    SPI_TypeDef *spi = s_spi->Instance;
    __HAL_SPI_ENABLE(s_spi);
    volatile uint16_t *const dr16 = (volatile uint16_t *)(uintptr_t)&spi->DR;
    while ((spi->SR & SPI_SR_FRLVL) != 0U) { (void)*dr16; }       /* 16-bit pops */
    (void)spi->SR;                            /* clears a possible OVR with the DR read */

    s_rx_dma        = s_spi->hdmarx->DmaBaseAddress;
    s_rx_flag_shift = s_spi->hdmarx->ChannelIndex & 0x1FU;
    s_spi->hdmarx->XferHalfCpltCallback = NULL;               /* no HT interrupt */
    if (HAL_DMA_Start_IT(s_spi->hdmarx, (uint32_t)&spi->DR, (uint32_t)s_adc_raw, 2U) != HAL_OK)
    {
        Error_Handler();
    }
    SET_BIT(spi->CR2, SPI_CR2_RXDMAEN);

    /* --- HRTIM Timer E DMA: dummy word -> SPI1->DR on CMP2 and CMP3 ------- */
    const HRTIM_TimerParamTypeDef *te = &s_hrtim->TimerParam[HRTIM_TIMERINDEX_TIMER_E];
    s_hrtim->hdmaTimerE->XferErrorCallback = hrtim_dma_error;
    __HAL_DMA_ENABLE_IT(s_hrtim->hdmaTimerE, DMA_IT_TE);      /* errors only    */
    if (HAL_DMA_Start(s_hrtim->hdmaTimerE, te->DMASrcAddress, te->DMADstAddress, te->DMASize) != HAL_OK)
    {
        Error_Handler();
    }
    __HAL_HRTIM_TIMER_ENABLE_DMA(s_hrtim, HRTIM_TIMERINDEX_TIMER_E, te->DMARequests);

    /* --- start: preload -> active, then A and E with one MCR write -------- */
    if (HAL_HRTIM_SoftwareUpdate(s_hrtim, HRTIM_TIMERUPDATE_A | HRTIM_TIMERUPDATE_E) != HAL_OK)
    {
        Error_Handler();
    }
    s_running = true;
    if (HAL_HRTIM_WaveformCountStart(s_hrtim, HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_E) != HAL_OK)
    {
        Error_Handler();
    }
}

void pwr_set_voltage(float volts)
{
    if (!(volts >= 0.0f))         { volts = 0.0f; }
    if (volts > PWR_VSET_MAX_V)   { volts = PWR_VSET_MAX_V; }
    s_vset = volts;
}

void pwr_set_current(float amps)
{
    if (!(amps >= 0.0f))          { amps = 0.0f; }
    if (amps > PWR_ISET_MAX_A)    { amps = PWR_ISET_MAX_A; }
    s_iset = amps;
}

bool pwr_output_on(void)
{
    bool ok = false;
    CRITICAL_ENTER();                            /* the ISR also writes the state */
    if (ocp_hw_active())
    {
        s_st.fault |= PWR_FAULT_OCP_HW;          /* above the trip level already  */
        s_st.state  = PWR_STATE_FAULT;
    }
    else if (s_running && (s_st.fault == PWR_FAULT_NONE))
    {
        if (s_st.state == PWR_STATE_OFF) { s_st.state = PWR_STATE_ARMED; }
        ok = true;
    }
    CRITICAL_EXIT();
    return ok;
}

void pwr_output_off(void)
{
    CRITICAL_ENTER();
    s_st.state = PWR_STATE_OFF;   /* first: the ISR will not re-enable anything */
    bridge_off();
    CRITICAL_EXIT();
}

void pwr_clear_fault(void)
{
    CRITICAL_ENTER();
    if (s_st.state == PWR_STATE_FAULT)
    {
        /* CONFIG, DMA and WATCHDOG faults stop the measurement chain and need
         * a reset (and, for CONFIG, a corrected .ioc) */
        uint32_t clr = PWR_FAULT_OVP | PWR_FAULT_OCP;
        if (!ocp_hw_active())                    /* current below the trip level */
        {
            bridge_off();                        /* TA1/TA2: FAULT -> IDLE state  */
            HRTIM1->sCommonRegs.ICR = HRTIM_ICR_FLT5C;
            clr |= PWR_FAULT_OCP_HW;
        }
        s_st.fault &= ~clr;
        if (s_st.fault == PWR_FAULT_NONE) { s_st.state = PWR_STATE_OFF; }
    }
    CRITICAL_EXIT();
}

float pwr_set_ocp_hw(float amps)
{
    if (s_dac == NULL) { return 0.0f; }
    if (!(amps >= 0.0f)) { amps = 0.0f; }

    float code = (amps * PWR_OCP_HW_SENSE_V_PER_A + PWR_OCP_HW_SENSE_OFFSET_V)
                 * (4096.0f / PWR_DAC_VREF_V);
    if (!(code >= 0.0f))  { code = 0.0f; }
    if (code > 4095.0f)   { code = 4095.0f; }
    const uint32_t c = (uint32_t)(code + 0.5f);              /* 0 ... 4095    */

    (void)HAL_DAC_SetValue(s_dac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, c);
    s_st.ocp_hw_dac = (uint16_t)c;
    s_st.ocp_hw_a   = ((float)c * (PWR_DAC_VREF_V / 4096.0f) - PWR_OCP_HW_SENSE_OFFSET_V)
                      / PWR_OCP_HW_SENSE_V_PER_A;
    return s_st.ocp_hw_a;
}

const pwr_status_t *pwr_status(void)
{
    return &s_st;
}

void pwr_tick_1ms(void)
{
    static uint32_t last_count;
    static uint32_t stalled_ms;

    if (!s_running) { stalled_ms = 0U; return; }

    const uint32_t n = s_st.isr_count;
    if (n == last_count)
    {
        if (++stalled_ms >= PWR_WATCHDOG_MS)
        {
            s_running = false;
            trip(PWR_FAULT_WATCHDOG);
        }
    }
    else
    {
        stalled_ms = 0U;
    }
    last_count = n;
}

void pwr_emergency_off(void)
{
    bridge_off();
}
