/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_MOTOR_B */
#define PWM_MOTOR_B_INST                                                  TIMG12
#define PWM_MOTOR_B_INST_IRQHandler                            TIMG12_IRQHandler
#define PWM_MOTOR_B_INST_INT_IRQN                              (TIMG12_INT_IRQn)
#define PWM_MOTOR_B_INST_CLK_FREQ                                       32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_MOTOR_B_C0_PORT                                           GPIOB
#define GPIO_PWM_MOTOR_B_C0_PIN                                   DL_GPIO_PIN_13
#define GPIO_PWM_MOTOR_B_C0_IOMUX                                (IOMUX_PINCM30)
#define GPIO_PWM_MOTOR_B_C0_IOMUX_FUNC              IOMUX_PINCM30_PF_TIMG12_CCP0
#define GPIO_PWM_MOTOR_B_C0_IDX                              DL_TIMER_CC_0_INDEX

/* Defines for PWM_MOTOR_A */
#define PWM_MOTOR_A_INST                                                   TIMA0
#define PWM_MOTOR_A_INST_IRQHandler                             TIMA0_IRQHandler
#define PWM_MOTOR_A_INST_INT_IRQN                               (TIMA0_INT_IRQn)
#define PWM_MOTOR_A_INST_CLK_FREQ                                       32000000
/* GPIO defines for channel 1 */
#define GPIO_PWM_MOTOR_A_C1_PORT                                           GPIOB
#define GPIO_PWM_MOTOR_A_C1_PIN                                   DL_GPIO_PIN_20
#define GPIO_PWM_MOTOR_A_C1_IOMUX                                (IOMUX_PINCM48)
#define GPIO_PWM_MOTOR_A_C1_IOMUX_FUNC               IOMUX_PINCM48_PF_TIMA0_CCP1
#define GPIO_PWM_MOTOR_A_C1_IDX                              DL_TIMER_CC_1_INDEX




/* Port definition for Pin Group GPIO_START_BUTTON */
#define GPIO_START_BUTTON_PORT                                           (GPIOB)

/* Defines for S2: GPIOB.21 with pinCMx 49 on package pin 20 */
#define GPIO_START_BUTTON_S2_PIN                                (DL_GPIO_PIN_21)
#define GPIO_START_BUTTON_S2_IOMUX                               (IOMUX_PINCM49)
/* Port definition for Pin Group GPIO_MOTOR_DIR */
#define GPIO_MOTOR_DIR_PORT                                              (GPIOB)

/* Defines for AIN1: GPIOB.6 with pinCMx 23 on package pin 58 */
#define GPIO_MOTOR_DIR_AIN1_PIN                                  (DL_GPIO_PIN_6)
#define GPIO_MOTOR_DIR_AIN1_IOMUX                                (IOMUX_PINCM23)
/* Defines for AIN2: GPIOB.7 with pinCMx 24 on package pin 59 */
#define GPIO_MOTOR_DIR_AIN2_PIN                                  (DL_GPIO_PIN_7)
#define GPIO_MOTOR_DIR_AIN2_IOMUX                                (IOMUX_PINCM24)
/* Defines for BIN1: GPIOB.16 with pinCMx 33 on package pin 4 */
#define GPIO_MOTOR_DIR_BIN1_PIN                                 (DL_GPIO_PIN_16)
#define GPIO_MOTOR_DIR_BIN1_IOMUX                                (IOMUX_PINCM33)
/* Defines for BIN2: GPIOB.15 with pinCMx 32 on package pin 3 */
#define GPIO_MOTOR_DIR_BIN2_PIN                                 (DL_GPIO_PIN_15)
#define GPIO_MOTOR_DIR_BIN2_IOMUX                                (IOMUX_PINCM32)
/* Defines for S1_RIGHT: GPIOA.24 with pinCMx 54 on package pin 25 */
#define GPIO_LINE_SENSOR_S1_RIGHT_PORT                                   (GPIOA)
#define GPIO_LINE_SENSOR_S1_RIGHT_PIN                           (DL_GPIO_PIN_24)
#define GPIO_LINE_SENSOR_S1_RIGHT_IOMUX                          (IOMUX_PINCM54)
/* Defines for S2_MID_RIGHT: GPIOA.22 with pinCMx 47 on package pin 18 */
#define GPIO_LINE_SENSOR_S2_MID_RIGHT_PORT                               (GPIOA)
#define GPIO_LINE_SENSOR_S2_MID_RIGHT_PIN                       (DL_GPIO_PIN_22)
#define GPIO_LINE_SENSOR_S2_MID_RIGHT_IOMUX                      (IOMUX_PINCM47)
/* Defines for S3_MID_LEFT: GPIOB.24 with pinCMx 52 on package pin 23 */
#define GPIO_LINE_SENSOR_S3_MID_LEFT_PORT                                (GPIOB)
#define GPIO_LINE_SENSOR_S3_MID_LEFT_PIN                        (DL_GPIO_PIN_24)
#define GPIO_LINE_SENSOR_S3_MID_LEFT_IOMUX                       (IOMUX_PINCM52)
/* Defines for S4_LEFT: GPIOA.25 with pinCMx 55 on package pin 26 */
#define GPIO_LINE_SENSOR_S4_LEFT_PORT                                    (GPIOA)
#define GPIO_LINE_SENSOR_S4_LEFT_PIN                            (DL_GPIO_PIN_25)
#define GPIO_LINE_SENSOR_S4_LEFT_IOMUX                           (IOMUX_PINCM55)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_MOTOR_B_init(void);
void SYSCFG_DL_PWM_MOTOR_A_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
