/*
 * TCAN4x5x_Reg.h
 * Description: This file contains the register definitions for the TCAN4x5x Family
 *
 * There are a few different define domains:
 *   - REG_MCAN_x: MCAN register address defines
 *   - MCAN_DLC_x: DLC values for the RX and TX FIFO element defines
 *   - REG_SPI_x : SPI Controller register address defines
 *   - REG_DEV_x : TCAN4x5x Device-specific register address defines
 *   - REG_BITS_x: Register bit defines in a similar manner as above.
 *   EX: REG_BITS_MCAN_CCCR_INIT is the hex value corresponding to the REG_MCAN_CCCR register's INIT bit
 *
 *
 *  Updated 10/12/2018
 *      - 1.0.1: Added Device IE Mask define
 *
 * Copyright (c) 2019 Texas Instruments Incorporated.  All rights reserved.
 * Software License Agreement
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */



#ifndef TCAN4X5X_REG_H_
#define TCAN4X5X_REG_H_

#define MRAM_SIZE                                   2048

//*****************************************************************************
// Register Address Sections
//*****************************************************************************
#define REG_SPI_CONFIG                              0x0000
#define REG_DEV_CONFIG                              0x0800
#define REG_MCAN                                    0x1000
#define REG_MRAM                                    0x8000


//*****************************************************************************
// SPI Registers and Device ID Register Addresses:  0x0000 Prefix
//*****************************************************************************
#define REG_SPI_DEVICE_ID0                          0x0000
#define REG_SPI_DEVICE_ID1                          0x0004
#define REG_SPI_REVISION                            0x0008
#define REG_SPI_STATUS                              0x000C
#define REG_SPI_ERROR_STATUS_MASK                   0x0010


//*****************************************************************************
// Device Configuration Register Addresses:     0x0800 Prefix
//*****************************************************************************
#define REG_DEV_MODES_AND_PINS                      0x0800
#define REG_DEV_TIMESTAMP_PRESCALER                 0x0804
#define REG_DEV_TEST_REGISTERS                      0x0808
#define REG_DEV_IR                                  0x0820
#define REG_DEV_IE                                  0x0830


//*****************************************************************************
// MCAN Register Addresses:     0x1000 Prefix
//*****************************************************************************
#define REG_MCAN_CREL                               0x1000
#define REG_MCAN_ENDN                               0x1004
#define REG_MCAN_CUST                               0x1008
#define REG_MCAN_DBTP                               0x100C
#define REG_MCAN_TEST                               0x1010
#define REG_MCAN_RWD                                0x1014
#define REG_MCAN_CCCR                               0x1018
#define REG_MCAN_NBTP                               0x101C
#define REG_MCAN_TSCC                               0x1020
#define REG_MCAN_TSCV                               0x1024
#define REG_MCAN_TOCC                               0x1028
#define REG_MCAN_TOCV                               0x102C
#define REG_MCAN_ECR                                0x1040
#define REG_MCAN_PSR                                0x1044
#define REG_MCAN_TDCR                               0x1048
#define REG_MCAN_IR                                 0x1050
#define REG_MCAN_IE                                 0x1054
#define REG_MCAN_ILS                                0x1058
#define REG_MCAN_ILE                                0x105C
#define REG_MCAN_GFC                                0x1080
#define REG_MCAN_SIDFC                              0x1084
#define REG_MCAN_XIDFC                              0x1088
#define REG_MCAN_XIDAM                              0x1090
#define REG_MCAN_HPMS                               0x1094
#define REG_MCAN_NDAT1                              0x1098
#define REG_MCAN_NDAT2                              0x109C
#define REG_MCAN_RXF0C                              0x10A0
#define REG_MCAN_RXF0S                              0x10A4
#define REG_MCAN_RXF0A                              0x10A8
#define REG_MCAN_RXBC                               0x10AC
#define REG_MCAN_RXF1C                              0x10B0
#define REG_MCAN_RXF1S                              0x10B4
#define REG_MCAN_RXF1A                              0x10B8
#define REG_MCAN_RXESC                              0x10BC
#define REG_MCAN_TXBC                               0x10C0
#define REG_MCAN_TXFQS                              0x10C4
#define REG_MCAN_TXESC                              0x10C8
#define REG_MCAN_TXBRP                              0x10CC
#define REG_MCAN_TXBAR                              0x10D0
#define REG_MCAN_TXBCR                              0x10D4
#define REG_MCAN_TXBTO                              0x10D8
#define REG_MCAN_TXBCF                              0x10DC
#define REG_MCAN_TXBTIE                             0x10E0
#define REG_MCAN_TXBCIE                             0x10E4
#define REG_MCAN_TXEFC                              0x10F0
#define REG_MCAN_TXEFS                              0x10F4
#define REG_MCAN_TXEFA                              0x10F8
//*****************************************************************************


//*****************************************************************************
// DLC Value Defines: Used for RX and TX elements. The DLC[3:0] bit field
//*****************************************************************************
#define MCAN_DLC_0B                                 0x00000000
#define MCAN_DLC_1B                                 0x00000001
#define MCAN_DLC_2B                                 0x00000002
#define MCAN_DLC_3B                                 0x00000003
#define MCAN_DLC_4B                                 0x00000004
#define MCAN_DLC_5B                                 0x00000005
#define MCAN_DLC_6B                                 0x00000006
#define MCAN_DLC_7B                                 0x00000007
#define MCAN_DLC_8B                                 0x00000008
#define MCAN_DLC_12B                                0x00000009
#define MCAN_DLC_16B                                0x0000000A
#define MCAN_DLC_20B                                0x0000000B
#define MCAN_DLC_24B                                0x0000000C
#define MCAN_DLC_32B                                0x0000000D
#define MCAN_DLC_48B                                0x0000000E
#define MCAN_DLC_64B                                0x0000000F
//*****************************************************************************




//*****************************************************************************
// MCAN Register Bit Field Defines
//*****************************************************************************

// DBTP
#define REG_BITS_MCAN_DBTP_TDC_EN                   0x00800000

// TEST
#define REG_BITS_MCAN_TEST_RX_DOM                   0x00000000
#define REG_BITS_MCAN_TEST_RX_REC                   0x00000080
#define REG_BITS_MCAN_TEST_TX_SP                    0x00000020
#define REG_BITS_MCAN_TEST_TX_DOM                   0x00000040
#define REG_BITS_MCAN_TEST_TX_REC                   0x00000060
#define REG_BITS_MCAN_TEST_LOOP_BACK                0x00000010

// CCCR
#define REG_BITS_MCAN_CCCR_RESERVED_MASK            0xFFFF0C00
#define REG_BITS_MCAN_CCCR_NISO_ISO                 0x00000000
#define REG_BITS_MCAN_CCCR_NISO_BOSCH               0x00008000
#define REG_BITS_MCAN_CCCR_TXP                      0x00004000
#define REG_BITS_MCAN_CCCR_EFBI                     0x00002000
#define REG_BITS_MCAN_CCCR_PXHD_DIS                 0x00001000
#define REG_BITS_MCAN_CCCR_BRSE                     0x00000200
#define REG_BITS_MCAN_CCCR_FDOE                     0x00000100
#define REG_BITS_MCAN_CCCR_TEST                     0x00000080
#define REG_BITS_MCAN_CCCR_DAR_DIS                  0x00000040
#define REG_BITS_MCAN_CCCR_MON                      0x00000020
#define REG_BITS_MCAN_CCCR_CSR                      0x00000010
#define REG_BITS_MCAN_CCCR_CSA                      0x00000008
#define REG_BITS_MCAN_CCCR_ASM                      0x00000004
#define REG_BITS_MCAN_CCCR_CCE                      0x00000002
#define REG_BITS_MCAN_CCCR_INIT                     0x00000001

// IE
#define REG_BITS_MCAN_IE_ARAE                       0x20000000
#define REG_BITS_MCAN_IE_PEDE                       0x10000000
#define REG_BITS_MCAN_IE_PEAE                       0x08000000
#define REG_BITS_MCAN_IE_WDIE                       0x04000000
#define REG_BITS_MCAN_IE_BOE                        0x02000000
#define REG_BITS_MCAN_IE_EWE                        0x01000000
#define REG_BITS_MCAN_IE_EPE                        0x00800000
#define REG_BITS_MCAN_IE_ELOE                       0x00400000
#define REG_BITS_MCAN_IE_BEUE                       0x00200000
#define REG_BITS_MCAN_IE_BECE                       0x00100000
#define REG_BITS_MCAN_IE_DRXE                       0x00080000
#define REG_BITS_MCAN_IE_TOOE                       0x00040000
#define REG_BITS_MCAN_IE_MRAFE                      0x00020000
#define REG_BITS_MCAN_IE_TSWE                       0x00010000
#define REG_BITS_MCAN_IE_TEFLE                      0x00008000
#define REG_BITS_MCAN_IE_TEFFE                      0x00004000
#define REG_BITS_MCAN_IE_TEFWE                      0x00002000
#define REG_BITS_MCAN_IE_TEFNE                      0x00001000
#define REG_BITS_MCAN_IE_TFEE                       0x00000800
#define REG_BITS_MCAN_IE_TCFE                       0x00000400
#define REG_BITS_MCAN_IE_TCE                        0x00000200
#define REG_BITS_MCAN_IE_HPME                       0x00000100
#define REG_BITS_MCAN_IE_RF1LE                      0x00000080
#define REG_BITS_MCAN_IE_RF1FE                      0x00000040
#define REG_BITS_MCAN_IE_RF1WE                      0x00000020
#define REG_BITS_MCAN_IE_RF1NE                      0x00000010
#define REG_BITS_MCAN_IE_RF0LE                      0x00000008
#define REG_BITS_MCAN_IE_RF0FE                      0x00000004
#define REG_BITS_MCAN_IE_RF0WE                      0x00000002
#define REG_BITS_MCAN_IE_RF0NE                      0x00000001

// IR
#define REG_BITS_MCAN_IR_ARA                        0x20000000
#define REG_BITS_MCAN_IR_PED                        0x10000000
#define REG_BITS_MCAN_IR_PEA                        0x08000000
#define REG_BITS_MCAN_IR_WDI                        0x04000000
#define REG_BITS_MCAN_IR_BO                         0x02000000
#define REG_BITS_MCAN_IR_EW                         0x01000000
#define REG_BITS_MCAN_IR_EP                         0x00800000
#define REG_BITS_MCAN_IR_ELO                        0x00400000
#define REG_BITS_MCAN_IR_BEU                        0x00200000
#define REG_BITS_MCAN_IR_BEC                        0x00100000
#define REG_BITS_MCAN_IR_DRX                        0x00080000
#define REG_BITS_MCAN_IR_TOO                        0x00040000
#define REG_BITS_MCAN_IR_MRAF                       0x00020000
#define REG_BITS_MCAN_IR_TSW                        0x00010000
#define REG_BITS_MCAN_IR_TEFL                       0x00008000
#define REG_BITS_MCAN_IR_TEFF                       0x00004000
#define REG_BITS_MCAN_IR_TEFW                       0x00002000
#define REG_BITS_MCAN_IR_TEFN                       0x00001000
#define REG_BITS_MCAN_IR_TFE                        0x00000800
#define REG_BITS_MCAN_IR_TCF                        0x00000400
#define REG_BITS_MCAN_IR_TC                         0x00000200
#define REG_BITS_MCAN_IR_HPM                        0x00000100
#define REG_BITS_MCAN_IR_RF1L                       0x00000080
#define REG_BITS_MCAN_IR_RF1F                       0x00000040
#define REG_BITS_MCAN_IR_RF1W                       0x00000020
#define REG_BITS_MCAN_IR_RF1N                       0x00000010
#define REG_BITS_MCAN_IR_RF0L                       0x00000008
#define REG_BITS_MCAN_IR_RF0F                       0x00000004
#define REG_BITS_MCAN_IR_RF0W                       0x00000002
#define REG_BITS_MCAN_IR_RF0N                       0x00000001

// ILS
#define REG_BITS_MCAN_IE_ARAL                       0x20000000
#define REG_BITS_MCAN_IE_PEDL                       0x10000000
#define REG_BITS_MCAN_IE_PEAL                       0x08000000
#define REG_BITS_MCAN_IE_WDIL                       0x04000000
#define REG_BITS_MCAN_IE_BOL                        0x02000000
#define REG_BITS_MCAN_IE_EWL                        0x01000000
#define REG_BITS_MCAN_IE_EPL                        0x00800000
#define REG_BITS_MCAN_IE_ELOL                       0x00400000
#define REG_BITS_MCAN_IE_BEUL                       0x00200000
#define REG_BITS_MCAN_IE_BECL                       0x00100000
#define REG_BITS_MCAN_IE_DRXL                       0x00080000
#define REG_BITS_MCAN_IE_TOOL                       0x00040000
#define REG_BITS_MCAN_IE_MRAFL                      0x00020000
#define REG_BITS_MCAN_IE_TSWL                       0x00010000
#define REG_BITS_MCAN_IE_TEFLL                      0x00008000
#define REG_BITS_MCAN_IE_TEFFL                      0x00004000
#define REG_BITS_MCAN_IE_TEFWL                      0x00002000
#define REG_BITS_MCAN_IE_TEFNL                      0x00001000
#define REG_BITS_MCAN_IE_TFEL                       0x00000800
#define REG_BITS_MCAN_IE_TCFL                       0x00000400
#define REG_BITS_MCAN_IE_TCL                        0x00000200
#define REG_BITS_MCAN_IE_HPML                       0x00000100
#define REG_BITS_MCAN_IE_RF1LL                      0x00000080
#define REG_BITS_MCAN_IE_RF1FL                      0x00000040
#define REG_BITS_MCAN_IE_RF1WL                      0x00000020
#define REG_BITS_MCAN_IE_RF1NL                      0x00000010
#define REG_BITS_MCAN_IE_RF0LL                      0x00000008
#define REG_BITS_MCAN_IE_RF0FL                      0x00000004
#define REG_BITS_MCAN_IE_RF0WL                      0x00000002
#define REG_BITS_MCAN_IE_RF0NL                      0x00000001

// ILE
#define REG_BITS_MCAN_ILE_EINT1                     0x00000002
#define REG_BITS_MCAN_ILE_EINT0                     0x00000001

// GFC
#define REG_BITS_MCAN_GFC_ANFS_FIFO0                0x00000000
#define REG_BITS_MCAN_GFC_ANFS_FIFO1                0x00000010
#define REG_BITS_MCAN_GFC_ANFE_FIFO0                0x00000000
#define REG_BITS_MCAN_GFC_ANFE_FIFO1                0x00000004
#define REG_BITS_MCAN_GFC_RRFS                      0x00000002
#define REG_BITS_MCAN_GFC_RRFE                      0x00000001
#define REG_BITS_MCAN_GFC_MASK                      0x0000003F

// NDAT1

// NDAT2

// RXF0C
#define REG_BITS_MCAN_RXF0C_F0OM_OVERWRITE          0x80000000

// RXESC
#define REG_BITS_MCAN_RXESC_RBDS_8B                 0x00000000
#define REG_BITS_MCAN_RXESC_RBDS_12B                0x00000100
#define REG_BITS_MCAN_RXESC_RBDS_16B                0x00000200
#define REG_BITS_MCAN_RXESC_RBDS_20B                0x00000300
#define REG_BITS_MCAN_RXESC_RBDS_24B                0x00000400
#define REG_BITS_MCAN_RXESC_RBDS_32B                0x00000500
#define REG_BITS_MCAN_RXESC_RBDS_48B                0x00000600
#define REG_BITS_MCAN_RXESC_RBDS_64B                0x00000700
#define REG_BITS_MCAN_RXESC_F1DS_8B                 0x00000000
#define REG_BITS_MCAN_RXESC_F1DS_12B                0x00000010
#define REG_BITS_MCAN_RXESC_F1DS_16B                0x00000020
#define REG_BITS_MCAN_RXESC_F1DS_20B                0x00000030
#define REG_BITS_MCAN_RXESC_F1DS_24B                0x00000040
#define REG_BITS_MCAN_RXESC_F1DS_32B                0x00000050
#define REG_BITS_MCAN_RXESC_F1DS_48B                0x00000060
#define REG_BITS_MCAN_RXESC_F1DS_64B                0x00000070
#define REG_BITS_MCAN_RXESC_F0DS_8B                 0x00000000
#define REG_BITS_MCAN_RXESC_F0DS_12B                0x00000001
#define REG_BITS_MCAN_RXESC_F0DS_16B                0x00000002
#define REG_BITS_MCAN_RXESC_F0DS_20B                0x00000003
#define REG_BITS_MCAN_RXESC_F0DS_24B                0x00000004
#define REG_BITS_MCAN_RXESC_F0DS_32B                0x00000005
#define REG_BITS_MCAN_RXESC_F0DS_48B                0x00000006
#define REG_BITS_MCAN_RXESC_F0DS_64B                0x00000007

// TXBC
#define REG_BITS_MCAN_TXBC_TFQM                     0x40000000

// TXESC
#define REG_BITS_MCAN_TXESC_TBDS_8                  0x00000000
#define REG_BITS_MCAN_TXESC_TBDS_12                 0x00000001
#define REG_BITS_MCAN_TXESC_TBDS_16                 0x00000002
#define REG_BITS_MCAN_TXESC_TBDS_20                 0x00000003
#define REG_BITS_MCAN_TXESC_TBDS_24                 0x00000004
#define REG_BITS_MCAN_TXESC_TBDS_32                 0x00000005
#define REG_BITS_MCAN_TXESC_TBDS_48                 0x00000006
#define REG_BITS_MCAN_TXESC_TBDS_64                 0x00000007

// TSCC
#define REG_BITS_MCAN_TSCC_PRESCALER_MASK           0x000F0000
#define REG_BITS_MCAN_TSCC_COUNTER_ALWAYS_0         0x00000000
#define REG_BITS_MCAN_TSCC_COUNTER_USE_TCP          0x00000001
#define REG_BITS_MCAN_TSCC_COUNTER_EXTERNAL         0x00000002

// TXBAR
#define REG_BITS_MCAN_TXBAR_AR31                    0x80000000
#define REG_BITS_MCAN_TXBAR_AR30                    0x40000000
#define REG_BITS_MCAN_TXBAR_AR29                    0x20000000
#define REG_BITS_MCAN_TXBAR_AR28                    0x10000000
#define REG_BITS_MCAN_TXBAR_AR27                    0x08000000
#define REG_BITS_MCAN_TXBAR_AR26                    0x04000000
#define REG_BITS_MCAN_TXBAR_AR25                    0x02000000
#define REG_BITS_MCAN_TXBAR_AR24                    0x01000000
#define REG_BITS_MCAN_TXBAR_AR23                    0x00800000
#define REG_BITS_MCAN_TXBAR_AR22                    0x00400000
#define REG_BITS_MCAN_TXBAR_AR21                    0x00200000
#define REG_BITS_MCAN_TXBAR_AR20                    0x00100000
#define REG_BITS_MCAN_TXBAR_AR19                    0x00080000
#define REG_BITS_MCAN_TXBAR_AR18                    0x00040000
#define REG_BITS_MCAN_TXBAR_AR17                    0x00020000
#define REG_BITS_MCAN_TXBAR_AR16                    0x00010000
#define REG_BITS_MCAN_TXBAR_AR15                    0x00008000
#define REG_BITS_MCAN_TXBAR_AR14                    0x00004000
#define REG_BITS_MCAN_TXBAR_AR13                    0x00002000
#define REG_BITS_MCAN_TXBAR_AR12                    0x00001000
#define REG_BITS_MCAN_TXBAR_AR11                    0x00000800
#define REG_BITS_MCAN_TXBAR_AR10                    0x00000400
#define REG_BITS_MCAN_TXBAR_AR9                     0x00000200
#define REG_BITS_MCAN_TXBAR_AR8                     0x00000100
#define REG_BITS_MCAN_TXBAR_AR7                     0x00000080
#define REG_BITS_MCAN_TXBAR_AR6                     0x00000040
#define REG_BITS_MCAN_TXBAR_AR5                     0x00000020
#define REG_BITS_MCAN_TXBAR_AR4                     0x00000010
#define REG_BITS_MCAN_TXBAR_AR3                     0x00000008
#define REG_BITS_MCAN_TXBAR_AR2                     0x00000004
#define REG_BITS_MCAN_TXBAR_AR1                     0x00000002
#define REG_BITS_MCAN_TXBAR_AR0                     0x00000001

// TXBCR
#define REG_BITS_MCAN_TXBCR_CR31                    0x80000000
#define REG_BITS_MCAN_TXBCR_CR30                    0x40000000
#define REG_BITS_MCAN_TXBCR_CR29                    0x20000000
#define REG_BITS_MCAN_TXBCR_CR28                    0x10000000
#define REG_BITS_MCAN_TXBCR_CR27                    0x08000000
#define REG_BITS_MCAN_TXBCR_CR26                    0x04000000
#define REG_BITS_MCAN_TXBCR_CR25                    0x02000000
#define REG_BITS_MCAN_TXBCR_CR24                    0x01000000
#define REG_BITS_MCAN_TXBCR_CR23                    0x00800000
#define REG_BITS_MCAN_TXBCR_CR22                    0x00400000
#define REG_BITS_MCAN_TXBCR_CR21                    0x00200000
#define REG_BITS_MCAN_TXBCR_CR20                    0x00100000
#define REG_BITS_MCAN_TXBCR_CR19                    0x00080000
#define REG_BITS_MCAN_TXBCR_CR18                    0x00040000
#define REG_BITS_MCAN_TXBCR_CR17                    0x00020000
#define REG_BITS_MCAN_TXBCR_CR16                    0x00010000
#define REG_BITS_MCAN_TXBCR_CR15                    0x00008000
#define REG_BITS_MCAN_TXBCR_CR14                    0x00004000
#define REG_BITS_MCAN_TXBCR_CR13                    0x00002000
#define REG_BITS_MCAN_TXBCR_CR12                    0x00001000
#define REG_BITS_MCAN_TXBCR_CR11                    0x00000800
#define REG_BITS_MCAN_TXBCR_CR10                    0x00000400
#define REG_BITS_MCAN_TXBCR_CR9                     0x00000200
#define REG_BITS_MCAN_TXBCR_CR8                     0x00000100
#define REG_BITS_MCAN_TXBCR_CR7                     0x00000080
#define REG_BITS_MCAN_TXBCR_CR6                     0x00000040
#define REG_BITS_MCAN_TXBCR_CR5                     0x00000020
#define REG_BITS_MCAN_TXBCR_CR4                     0x00000010
#define REG_BITS_MCAN_TXBCR_CR3                     0x00000008
#define REG_BITS_MCAN_TXBCR_CR2                     0x00000004
#define REG_BITS_MCAN_TXBCR_CR1                     0x00000002
#define REG_BITS_MCAN_TXBCR_CR0                     0x00000001

// TXBTIE
#define REG_BITS_MCAN_TXBTIE_TIE31                  0x80000000
#define REG_BITS_MCAN_TXBTIE_TIE30                  0x40000000
#define REG_BITS_MCAN_TXBTIE_TIE29                  0x20000000
#define REG_BITS_MCAN_TXBTIE_TIE28                  0x10000000
#define REG_BITS_MCAN_TXBTIE_TIE27                  0x08000000
#define REG_BITS_MCAN_TXBTIE_TIE26                  0x04000000
#define REG_BITS_MCAN_TXBTIE_TIE25                  0x02000000
#define REG_BITS_MCAN_TXBTIE_TIE24                  0x01000000
#define REG_BITS_MCAN_TXBTIE_TIE23                  0x00800000
#define REG_BITS_MCAN_TXBTIE_TIE22                  0x00400000
#define REG_BITS_MCAN_TXBTIE_TIE21                  0x00200000
#define REG_BITS_MCAN_TXBTIE_TIE20                  0x00100000
#define REG_BITS_MCAN_TXBTIE_TIE19                  0x00080000
#define REG_BITS_MCAN_TXBTIE_TIE18                  0x00040000
#define REG_BITS_MCAN_TXBTIE_TIE17                  0x00020000
#define REG_BITS_MCAN_TXBTIE_TIE16                  0x00010000
#define REG_BITS_MCAN_TXBTIE_TIE15                  0x00008000
#define REG_BITS_MCAN_TXBTIE_TIE14                  0x00004000
#define REG_BITS_MCAN_TXBTIE_TIE13                  0x00002000
#define REG_BITS_MCAN_TXBTIE_TIE12                  0x00001000
#define REG_BITS_MCAN_TXBTIE_TIE11                  0x00000800
#define REG_BITS_MCAN_TXBTIE_TIE10                  0x00000400
#define REG_BITS_MCAN_TXBTIE_TIE9                   0x00000200
#define REG_BITS_MCAN_TXBTIE_TIE8                   0x00000100
#define REG_BITS_MCAN_TXBTIE_TIE7                   0x00000080
#define REG_BITS_MCAN_TXBTIE_TIE6                   0x00000040
#define REG_BITS_MCAN_TXBTIE_TIE5                   0x00000020
#define REG_BITS_MCAN_TXBTIE_TIE4                   0x00000010
#define REG_BITS_MCAN_TXBTIE_TIE3                   0x00000008
#define REG_BITS_MCAN_TXBTIE_TIE2                   0x00000004
#define REG_BITS_MCAN_TXBTIE_TIE1                   0x00000002
#define REG_BITS_MCAN_TXBTIE_TIE0                   0x00000001

// TXBCIE
#define REG_BITS_MCAN_TXBCIE_CFIE31                 0x80000000
#define REG_BITS_MCAN_TXBCIE_CFIE30                 0x40000000
#define REG_BITS_MCAN_TXBCIE_CFIE29                 0x20000000
#define REG_BITS_MCAN_TXBCIE_CFIE28                 0x10000000
#define REG_BITS_MCAN_TXBCIE_CFIE27                 0x08000000
#define REG_BITS_MCAN_TXBCIE_CFIE26                 0x04000000
#define REG_BITS_MCAN_TXBCIE_CFIE25                 0x02000000
#define REG_BITS_MCAN_TXBCIE_CFIE24                 0x01000000
#define REG_BITS_MCAN_TXBCIE_CFIE23                 0x00800000
#define REG_BITS_MCAN_TXBCIE_CFIE22                 0x00400000
#define REG_BITS_MCAN_TXBCIE_CFIE21                 0x00200000
#define REG_BITS_MCAN_TXBCIE_CFIE20                 0x00100000
#define REG_BITS_MCAN_TXBCIE_CFIE19                 0x00080000
#define REG_BITS_MCAN_TXBCIE_CFIE18                 0x00040000
#define REG_BITS_MCAN_TXBCIE_CFIE17                 0x00020000
#define REG_BITS_MCAN_TXBCIE_CFIE16                 0x00010000
#define REG_BITS_MCAN_TXBCIE_CFIE15                 0x00008000
#define REG_BITS_MCAN_TXBCIE_CFIE14                 0x00004000
#define REG_BITS_MCAN_TXBCIE_CFIE13                 0x00002000
#define REG_BITS_MCAN_TXBCIE_CFIE12                 0x00001000
#define REG_BITS_MCAN_TXBCIE_CFIE11                 0x00000800
#define REG_BITS_MCAN_TXBCIE_CFIE10                 0x00000400
#define REG_BITS_MCAN_TXBCIE_CFIE9                  0x00000200
#define REG_BITS_MCAN_TXBCIE_CFIE8                  0x00000100
#define REG_BITS_MCAN_TXBCIE_CFIE7                  0x00000080
#define REG_BITS_MCAN_TXBCIE_CFIE6                  0x00000040
#define REG_BITS_MCAN_TXBCIE_CFIE5                  0x00000020
#define REG_BITS_MCAN_TXBCIE_CFIE4                  0x00000010
#define REG_BITS_MCAN_TXBCIE_CFIE3                  0x00000008
#define REG_BITS_MCAN_TXBCIE_CFIE2                  0x00000004
#define REG_BITS_MCAN_TXBCIE_CFIE1                  0x00000002
#define REG_BITS_MCAN_TXBCIE_CFIE0                  0x00000001
//*****************************************************************************


//*****************************************************************************
// Device Register Bit Field Defines
//*****************************************************************************

// Modes of Operation and Pin Configuration Registers (0x0800)
// Generic masks
#define REG_BITS_DEVICE_MODE_FORCED_SET_BITS        0x00000020

// Wake pin
#define REG_BITS_DEVICE_MODE_WAKE_PIN_MASK          0xC0000000
#define REG_BITS_DEVICE_MODE_WAKE_PIN_DIS           0x00000000
#define REG_BITS_DEVICE_MODE_WAKE_PIN_RISING        0x40000000
#define REG_BITS_DEVICE_MODE_WAKE_PIN_FALLING       0x80000000
#define REG_BITS_DEVICE_MODE_WAKE_PIN_BOTHEDGES     0xC0000000

// WD_Timer (If applicable)
#define REG_BITS_DEVICE_MODE_WD_TIMER_MASK          0x30000000
#define REG_BITS_DEVICE_MODE_WD_TIMER_60MS          0x00000000
#define REG_BITS_DEVICE_MODE_WD_TIMER_600MS         0x10000000
#define REG_BITS_DEVICE_MODE_WD_TIMER_3S            0x20000000
#define REG_BITS_DEVICE_MODE_WD_TIMER_6S            0x30000000

// WD_TIMER_CLK_REF
#define REG_BITS_DEVICE_MODE_WD_CLK_MASK            0x08000000
#define REG_BITS_DEVICE_MODE_WD_CLK_20MHZ           0x00000000
#define REG_BITS_DEVICE_MODE_WD_CLK_40MHZ           0x08000000

// GPO2 Pin Configuration
#define REG_BITS_DEVICE_MODE_GPO2_MASK              0x00C00000
#define REG_BITS_DEVICE_MODE_GPO2_CAN_FAULT         0x00000000
#define REG_BITS_DEVICE_MODE_GPO2_MCAN_INT0         0x00400000
#define REG_BITS_DEVICE_MODE_GPO2_WDT               0x00800000
#define REG_BITS_DEVICE_MODE_GPO2_NINT              0x00C00000

// Test Mode Enable Bit
#define REG_BITS_DEVICE_MODE_TESTMODE_ENMASK        0x00200000
#define REG_BITS_DEVICE_MODE_TESTMODE_EN            0x00200000
#define REG_BITS_DEVICE_MODE_TESTMODE_DIS           0x00000000

// nWKRQ Pin GPO Voltage Rail COnfiguration
#define REG_BITS_DEVICE_MODE_NWKRQ_VOLT_MASK        0x00080000
#define REG_BITS_DEVICE_MODE_NWKRQ_VOLT_INTERNAL    0x00000000
#define REG_BITS_DEVICE_MODE_NWKRQ_VOLT_VIO         0x00080000

// WD_BIT
#define REG_BITS_DEVICE_MODE_WDT_RESET_BIT          0x00040000

// WD_ACTION
#define REG_BITS_DEVICE_MODE_WDT_ACTION_MASK        0x00020000
#define REG_BITS_DEVICE_MODE_WDT_ACTION_INT         0x00000000
#define REG_BITS_DEVICE_MODE_WDT_ACTION_INH_PULSE   0x00010000
#define REG_BITS_DEVICE_MODE_WDT_ACTION_WDT_PULSE   0x00020000

// CLKOUT/GPO1 Pin Mode Select
#define REG_BITS_DEVICE_MODE_GPO1_MODE_MASK         0x0000C000
#define REG_BITS_DEVICE_MODE_GPO1_MODE_GPO          0x00000000
#define REG_BITS_DEVICE_MODE_GPO1_MODE_CLKOUT       0x00004000
#define REG_BITS_DEVICE_MODE_GPO1_MODE_GPI          0x00008000

// Fail Safe Enable
#define REG_BITS_DEVICE_MODE_FAIL_SAFE_MASK         0x00002000
#define REG_BITS_DEVICE_MODE_FAIL_SAFE_EN           0x00002000
#define REG_BITS_DEVICE_MODE_FAIL_SAFE_DIS          0x00000000

// CLKOUT Prescaler
#define REG_BITS_DEVICE_MODE_CLKOUT_MASK            0x00001000
#define REG_BITS_DEVICE_MODE_CLKOUT_DIV1            0x00000000
#define REG_BITS_DEVICE_MODE_CLKOUT_DIV2            0x00001000

// GPO1 Function Select
#define REG_BITS_DEVICE_MODE_GPO1_FUNC_MASK         0x00000C00
#define REG_BITS_DEVICE_MODE_GPO1_FUNC_SPI_INT      0x00000000
#define REG_BITS_DEVICE_MODE_GPO1_FUNC_MCAN_INT1    0x00000400
#define REG_BITS_DEVICE_MODE_GPO1_FUNC_UVLO_THERM   0x00000800

// INH Pin Disable
#define REG_BITS_DEVICE_MODE_INH_MASK               0x00000200
#define REG_BITS_DEVICE_MODE_INH_DIS                0x00000200
#define REG_BITS_DEVICE_MODE_INH_EN                 0x00000000

// nWKRQ Pin Configuration
#define REG_BITS_DEVICE_MODE_NWKRQ_CONFIG_MASK      0x00000100
#define REG_BITS_DEVICE_MODE_NWKRQ_CONFIG_INH       0x00000000
#define REG_BITS_DEVICE_MODE_NWKRQ_CONFIG_WKRQ      0x00000100

// Mode of Operation
#define REG_BITS_DEVICE_MODE_DEVICEMODE_MASK        0x000000C0
#define REG_BITS_DEVICE_MODE_DEVICEMODE_SLEEP       0x00000000
#define REG_BITS_DEVICE_MODE_DEVICEMODE_STANDBY     0x00000040
#define REG_BITS_DEVICE_MODE_DEVICEMODE_NORMAL      0x00000080

// WDT_EN
#define REG_BITS_DEVICE_MODE_WDT_MASK               0x00000008
#define REG_BITS_DEVICE_MODE_WDT_EN                 0x00000008
#define REG_BITS_DEVICE_MODE_WDT_DIS                0x00000000

// Dev Reset
#define REG_BITS_DEVICE_MODE_DEVICE_RESET           0x00000004

// SWE_DIS: Sleep Wake Error Disable
#define REG_BITS_DEVICE_MODE_SWE_MASK               0x00000002
#define REG_BITS_DEVICE_MODE_SWE_DIS                0x00000002
#define REG_BITS_DEVICE_MODE_SWE_EN                 0x00000000

// Test Mode Configuration
#define REG_BITS_DEVICE_MODE_TESTMODE_MASK          0x00000001
#define REG_BITS_DEVICE_MODE_TESTMODE_PHY           0x00000000
#define REG_BITS_DEVICE_MODE_TESTMODE_CONTROLLER    0x00000001


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Device Interrupt Register values (0x0820)
#define REG_BITS_DEVICE_IR_CANLGND                  0x08000000
#define REG_BITS_DEVICE_IR_CANBUSOPEN               0x04000000
#define REG_BITS_DEVICE_IR_CANBUSGND                0x02000000
#define REG_BITS_DEVICE_IR_CANBUSBAT                0x01000000
//Reserved                                          0x00800000
#define REG_BITS_DEVICE_IR_UVSUP                    0x00400000
#define REG_BITS_DEVICE_IR_UVIO                     0x00200000
#define REG_BITS_DEVICE_IR_PWRON                    0x00100000
#define REG_BITS_DEVICE_IR_TSD                      0x00080000
#define REG_BITS_DEVICE_IR_WDTO                     0x00040000
//Reserved                                          0x00020000
#define REG_BITS_DEVICE_IR_ECCERR                   0x00010000
#define REG_BITS_DEVICE_IR_CANINT                   0x00008000
#define REG_BITS_DEVICE_IR_LWU                      0x00004000
#define REG_BITS_DEVICE_IR_WKERR                    0x00002000
#define REG_BITS_DEVICE_IR_FRAME_OVF                0x00001000
//Reserved                                          0x00000800
#define REG_BITS_DEVICE_IR_CANSLNT                  0x00000400
//Reserved                                          0x00000200
#define REG_BITS_DEVICE_IR_CANDOM                   0x00000100
#define REG_BITS_DEVICE_IR_GLOBALERR                0x00000080
#define REG_BITS_DEVICE_IR_nWKRQ                    0x00000040
#define REG_BITS_DEVICE_IR_CANERR                   0x00000020
#define REG_BITS_DEVICE_IR_CANBUSFAULT              0x00000010
#define REG_BITS_DEVICE_IR_SPIERR                   0x00000008
#define REG_BITS_DEVICE_IR_SWERR                    0x00000004
#define REG_BITS_DEVICE_IR_M_CAN_INT                0x00000002
#define REG_BITS_DEVICE_IR_VTWD                     0x00000001

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Device Interrupt Enable Values (0x0830)
#define REG_BITS_DEVICE_IE_UVCCOUT                  0x00800000
#define REG_BITS_DEVICE_IE_UVSUP                    0x00400000
#define REG_BITS_DEVICE_IE_UVIO                     0x00200000
#define REG_BITS_DEVICE_IE_PWRON                    0x00100000
#define REG_BITS_DEVICE_IE_TSD                      0x00080000
#define REG_BITS_DEVICE_IE_WDTO                     0x00040000
// Reserved
#define REG_BITS_DEVICE_IE_ECCERR                   0x00010000
#define REG_BITS_DEVICE_IE_CANINT                   0x00008000
#define REG_BITS_DEVICE_IE_LWU                      0x00004000
#define REG_BITS_DEVICE_IE_WKERR                    0x00002000
#define REG_BITS_DEVICE_IE_FRAME_OVF                0x00001000
//Reserved                                          0x00000800
#define REG_BITS_DEVICE_IE_CANSLNT                  0x00000400
//Reserved                                          0x00000200
#define REG_BITS_DEVICE_IE_CANDOM                   0x00000100
// Reserved                                         0x80-00
//#define REG_BITS_DEVICE_IE_MASK                     0xFF69D700
#define REG_BITS_DEVICE_IE_MASK                     0x0069D700


#endif /* TCAN4X5X_REG_H_ */
