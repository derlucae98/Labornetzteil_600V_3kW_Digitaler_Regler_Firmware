/**
 * @file    pwr_config.h
 * @brief   Control parameters of the digital full-bridge controller.
 *
 * Everything that is hardware configuration lives in the CubeMX project
 * (Labornetzteil_600V_3kW_Digitaler_Regler.ioc):
 *   - switching period, compare values (sampling / read instants)  -> HRTIM1
 *   - SPI clock, data size, clock phase                             -> SPI1
 *   - DMA channels, priorities                                     -> DMA
 *   - pins and labels                                               -> Pinout
 * pwr_init() reads those settings back from the peripherals and checks them
 * against the limits below (fault PWR_FAULT_CONFIG if they do not fit).
 *
 * This file only holds what CubeMX cannot know: control parameters, scaling,
 * protection limits and timing margins.
 */
#ifndef PWR_CONFIG_H
#define PWR_CONFIG_H

/* ========================================================================== */
/* 1. DMA channel of SPI1_RX (CubeMX: DMA Settings -> SPI1_RX -> Channel)      */
/*    The control ISR is this channel's IRQ handler. In CubeMX, NVIC -> Code  */
/*    generation, "Generate IRQ handler" is unchecked for this channel.       */
/* ========================================================================== */
#define PWR_ADC_DMA_CHANNEL     DMA1_Channel2
#define PWR_ADC_DMA_IRQHandler  DMA1_Channel2_IRQHandler

/* ========================================================================== */
/* 2. Duty cycle limits                                                       */
/*    d = on-time of ONE diagonal relative to HALF a switching period.        */
/* ========================================================================== */
#define PWR_DUTY_MAX_PERMILLE   750     /* max d in 1/1000; the remaining gap of
                                           each half period hosts the ADC sample */
#define PWR_TON_MIN_NS          30      /* minimum pulse (HRTIM limit 17.7 ns)  */

/* ========================================================================== */
/* 3. ADC timing margins (checked against the HRTIM Timer E compare values)   */
/* ========================================================================== */
#define PWR_ADC_TCNV_NS         800     /* CMP1 -> CMP2 >= tCNV,max of the
                                           MCP33131-10 (710 ns @85 C, 750 @125 C) */
#define PWR_SAMPLE_SETTLE_NS    300     /* latest turn-off edge -> sampling     */
#define PWR_ADC_DMA_MARGIN_NS   250     /* CMP2 -> CMP3 >= frame + this margin  */
#define PWR_COMPUTE_BUDGET_NS   2500    /* time the ISR needs before period end */

/* ========================================================================== */
/* 4. ADC scaling: physical = raw / 65536 * FULLSCALE + OFFSET                */
/*    raw[0] = ADC on CNVST_V (TE1, PC8), raw[1] = ADC on CNVST_I (TE2, PC9)  */
/*    PLACEHOLDERS - set them to your divider / shunt amplifier!              */
/* ========================================================================== */
#define PWR_V_FULLSCALE_V       650.0f  /* output voltage at ADC code 65536     */
#define PWR_V_OFFSET_V          0.0f
#define PWR_I_FULLSCALE_A       12.5f   /* output current at ADC code 65536     */
#define PWR_I_OFFSET_A          0.0f

/* MCP33131 recalibration at start-up (1024 SCLKs, ~0.5 s). Required by the
 * data sheet if VREF was not stable within ~64 ms after AVDD.               */
#define PWR_ADC_RECAL_AT_INIT   1
#define PWR_ADC_RECAL_DELAY_MS  100     /* let VREF/AVDD settle first           */
#define PWR_ADC_RECAL_TIMEOUT_MS 1000

/* ========================================================================== */
/* 5. Set-point limits, soft start, software protection                       */
/* ========================================================================== */
#define PWR_VSET_MAX_V          600.0f
#define PWR_ISET_MAX_A          10.0f
#define PWR_VREF_SLEW_V_PER_S   1000.0f /* reference ramp (1 V/ms), 0 = step    */
#define PWR_OVP_V               630.0f  /* trip: all bridge outputs off         */
#define PWR_OCP_A               11.0f
#define PWR_WATCHDOG_MS         2U      /* no control ISR for this long -> trip */

/* ========================================================================== */
/* 6. 3p3z compensators                                                       */
/*                                                                            */
/*    u[n] = B0*e[n] + B1*e[n-1] + B2*e[n-2] + B3*e[n-3]                      */
/*                   + A1*u[n-1] + A2*u[n-2] + A3*u[n-3]                      */
/*                                                                            */
/*           B0 + B1 z^-1 + B2 z^-2 + B3 z^-3                                 */
/*    H(z) = ---------------------------------                                */
/*           1  - A1 z^-1 - A2 z^-2 - A3 z^-3                                 */
/*                                                                            */
/*    NOTE THE SIGN: from MATLAB/Octave/Python c2d(..., Ts, 'tustin') with    */
/*    den = [1 a1 a2 a3]:  A1 = -a1, A2 = -a2, A3 = -a3;  B0..B3 = num.       */
/*                                                                            */
/*    e = reference - measurement [V] or [A], u = duty d (0 ... DUTY_MAX).    */
/*    Ts = HRTIM period (10 us at 100 kHz).                                   */
/*                                                                            */
/*    Defaults: slow pure integrators (u += B0*e) as safe placeholders.       */
/*    Crossover f_c ~= B0 * G * f_s / (2*pi), G = plant gain per unit duty    */
/*    (CV: V_out/d ~ n*V_in, CC: I_out/d ~ n*V_in/R_load).                     */
/*    CV: B0 = 1e-5 -> ~110 Hz at G = 700 V;  CC: B0 = 5e-4 -> ~95 Hz at 12 A. */
/* ========================================================================== */
#define PWR_CV_B0               1.0e-5f
#define PWR_CV_B1               0.0f
#define PWR_CV_B2               0.0f
#define PWR_CV_B3               0.0f
#define PWR_CV_A1               1.0f
#define PWR_CV_A2               0.0f
#define PWR_CV_A3               0.0f

#define PWR_CC_B0               5.0e-4f
#define PWR_CC_B1               0.0f
#define PWR_CC_B2               0.0f
#define PWR_CC_B3               0.0f
#define PWR_CC_A1               1.0f
#define PWR_CC_A2               0.0f
#define PWR_CC_A3               0.0f

/* ========================================================================== */
/* 7. Bring-up helpers                                                        */
/* ========================================================================== */
#define PWR_OPEN_LOOP           0       /* 1: ignore both loops, fixed duty      */
#define PWR_OPEN_LOOP_PERMILLE  100     /* fixed duty in open-loop mode (1/1000) */
#define PWR_DEBUG_PIN_ENABLE    1       /* DBG_ISR pin high while the ISR runs   */

/* ========================================================================== */
/* Compile-time sanity checks                                                 */
/* ========================================================================== */
#if (PWR_DUTY_MAX_PERMILLE > 950) || (PWR_DUTY_MAX_PERMILLE < 1)
#error "PWR_DUTY_MAX_PERMILLE out of range"
#endif
#if (PWR_TON_MIN_NS < 18)
#error "HRTIM compare values must be >= 0x60 ticks (17.7 ns)"
#endif
#if (PWR_ADC_TCNV_NS < 710)
#error "PWR_ADC_TCNV_NS shorter than tCNV,max of the MCP33131-10"
#endif

#endif /* PWR_CONFIG_H */
