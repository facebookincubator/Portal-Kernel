/*
* iaxxx-register-defs-ioctrl.h -- IAXXX register defination for ioctrl
*
* Copyright (c) 2016 Knowles Corporation.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 and
* only version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/

#ifndef __IAXXX_REGISTER_DEFS_IOCTRL_H__
#define __IAXXX_REGISTER_DEFS_IOCTRL_H__

/*** The base address for this set of registers ***/
#define IAXXX_IO_CTRL_REGS_ADDR (0x50022000)

/*** IO_CTRL_CDC_I2C_SCL (0x50022000) ***/
/* This register is used for pad CDC_I2C_SCL's IO function control.
Refer to the */
/* <a href="#io_pad_control_diagram">IO Function and PAD Control
diagram</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_CDC_I2C_SCL_ADDR (0x50022000)
#define IAXXX_IO_CTRL_CDC_I2C_SCL_MASK_VAL 0x01c1c01f
#define IAXXX_IO_CTRL_CDC_I2C_SCL_RMASK_VAL 0x01c1c01f
#define IAXXX_IO_CTRL_CDC_I2C_SCL_WMASK_VAL 0x01c1c01f
#define IAXXX_IO_CTRL_CDC_I2C_SCL_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad CDC_I2C_SCL.
*   0x9: i2c4_clk
*   0xb: spi1_cs0_n
*   0xc: spi0_cs1_n
*   0xd: spi1_cs1_n
*   0xe: spi2_cs1_n
*   0x10: fo_6
*   0x12: gpio_5
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_CDC_I2C_SCL_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_CDC_I2C_SCL_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_CDC_I2C_SCL_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_CDC_I2C_SCL_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_CDC_I2C_SCL_MUX_SEL_DECL (4:0)

/*
* Function i2c4_clk can be selected through the following pads:
*   COMMC_2
*   COMMF_2
*   PORTA_DI
*   CDC_PWRON
*   CDC_I2C_SCL
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C4_CLK_AND_SEL for pad CDC_I2C_SCL, a
* separate receiver enable for the I2C4_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_I2C_SCL_I2C4_CLK_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_CDC_I2C_SCL_I2C4_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_I2C_SCL_I2C4_CLK_AND_SEL_POS 14
#define IAXXX_IO_CTRL_CDC_I2C_SCL_I2C4_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_I2C_SCL_I2C4_CLK_AND_SEL_DECL 14

/*
* Function uart0_cts_n can be selected through the following pads:
*   COMMA_0
*   COMMC_0
*   CDC_I2C_SCL
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control UART0_CTS_N_AND_SEL for pad CDC_I2C_SCL, a
* separate receiver enable for the UART0_CTS_N pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_I2C_SCL_UART0_CTS_N_AND_SEL_MASK 0x00008000
#define IAXXX_IO_CTRL_CDC_I2C_SCL_UART0_CTS_N_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_I2C_SCL_UART0_CTS_N_AND_SEL_POS 15
#define IAXXX_IO_CTRL_CDC_I2C_SCL_UART0_CTS_N_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_I2C_SCL_UART0_CTS_N_AND_SEL_DECL 15

/*
* Function spi1_cs0_n can be selected through the following pads:
*   COMMB_2
*   PORTE_DI
*   CDC_INT_N
*   CDC_I2C_SCL
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SPI1_CS0_N_AND_SEL for pad CDC_I2C_SCL, a
* separate receiver enable for the SPI1_CS0_N pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_I2C_SCL_SPI1_CS0_N_AND_SEL_MASK 0x00010000
#define IAXXX_IO_CTRL_CDC_I2C_SCL_SPI1_CS0_N_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_I2C_SCL_SPI1_CS0_N_AND_SEL_POS 16
#define IAXXX_IO_CTRL_CDC_I2C_SCL_SPI1_CS0_N_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_I2C_SCL_SPI1_CS0_N_AND_SEL_DECL 16

/*
* Function fi_6 can be selected through the following pads:
*   COMMB_2
*   CDC_I2C_SCL
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control FI_6_AND_SEL for pad CDC_I2C_SCL, a separate
* receiver enable for the FI_6 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_I2C_SCL_FI_6_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_CDC_I2C_SCL_FI_6_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_I2C_SCL_FI_6_AND_SEL_POS 22
#define IAXXX_IO_CTRL_CDC_I2C_SCL_FI_6_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_I2C_SCL_FI_6_AND_SEL_DECL 22

/*
* This field is used to control GPIO_5_AND_SEL for pad CDC_I2C_SCL, a
* separate receiver enable for the GPIO_5 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_I2C_SCL_GPIO_5_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_CDC_I2C_SCL_GPIO_5_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_CDC_I2C_SCL_GPIO_5_AND_SEL_POS 23
#define IAXXX_IO_CTRL_CDC_I2C_SCL_GPIO_5_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_I2C_SCL_GPIO_5_AND_SEL_DECL 23

/*
* Function io2stmr_5_13_21_29 can be selected through the following pads:
*   COMMB_1
*   PORTB_FS
*   PORTD_FS
*   CDC_I2C_SCL
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_5_13_21_29_AND_SEL for pad
* CDC_I2C_SCL, a separate receiver enable for the IO2STMR_5_13_21_29 pad
* function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_I2C_SCL_IO2STMR_5_13_21_29_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_CDC_I2C_SCL_IO2STMR_5_13_21_29_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_I2C_SCL_IO2STMR_5_13_21_29_AND_SEL_POS 24
#define IAXXX_IO_CTRL_CDC_I2C_SCL_IO2STMR_5_13_21_29_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_I2C_SCL_IO2STMR_5_13_21_29_AND_SEL_DECL 24

/*** IO_CTRL_CDC_I2C_SDA_RD (0x50022004) ***/
/* This register is used for pad CDC_I2C_SDA_RD's IO function control.
Refer to */
/* the <a href="#io_pad_control_diagram">IO Function and
PAD Control</a> */
/* to understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_ADDR (0x50022004)
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_MASK_VAL 0x01c1401f
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_RMASK_VAL 0x01c1401f
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_WMASK_VAL 0x01c1401f
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad CDC_I2C_SDA_RD.
*   0x9: i2c4_data
*   0xa: uart0_sout
*   0xb: spi1_clk
*   0xc: spi0_cs2_n
*   0xd: spi1_cs2_n
*   0xe: spi2_cs2_n
*   0x10: fo_5
*   0x12: gpio_7
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_MUX_SEL_DECL (4:0)

/*
* Function i2c4_data can be selected through the following pads:
*   COMMC_3
*   COMMF_3
*   PORTA_DO
*   CDC_I2C_SDA_RD
*   CDC_I2C_SDA_WR
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C4_DATA_AND_SEL for pad CDC_I2C_SDA_RD, a
* separate receiver enable for the I2C4_DATA pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_I2C4_DATA_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_I2C4_DATA_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_I2C4_DATA_AND_SEL_POS 14
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_I2C4_DATA_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_I2C4_DATA_AND_SEL_DECL 14

/*
* Function spi1_clk can be selected through the following pads:
*   COMMB_1
*   PORTE_FS
*   CDC_I2C_SDA_RD
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
* This field is used to control SPI1_CLK_AND_SEL for pad CDC_I2C_SDA_RD, a
* separate receiver enable for the SPI1_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_SPI1_CLK_AND_SEL_MASK 0x00010000
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_SPI1_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_SPI1_CLK_AND_SEL_POS 16
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_SPI1_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_SPI1_CLK_AND_SEL_DECL 16

/*
* Function fi_5 can be selected through the following pads:
*   COMMB_1
*   CDC_I2C_SDA_RD
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
* This field is used to control FI_5_AND_SEL for pad CDC_I2C_SDA_RD, a
* separate receiver enable for the FI_5 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_FI_5_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_FI_5_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_FI_5_AND_SEL_POS 22
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_FI_5_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_FI_5_AND_SEL_DECL 22

/*
* This field is used to control GPIO_7_AND_SEL for pad CDC_I2C_SDA_RD, a
* separate receiver enable for the GPIO_7 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_GPIO_7_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_GPIO_7_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_GPIO_7_AND_SEL_POS 23
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_GPIO_7_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_GPIO_7_AND_SEL_DECL 23

/*
* Function io2stmr_7_15_23_31 can be selected through the following pads:
*   COMMB_3
*   PORTB_DO
*   PORTD_DO
*   CDC_I2C_SDA_RD
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
* This field is used to control IO2STMR_7_15_23_31_AND_SEL for pad
* CDC_I2C_SDA_RD, a separate receiver enable for the IO2STMR_7_15_23_31 pad
* function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_IO2STMR_7_15_23_31_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_IO2STMR_7_15_23_31_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_IO2STMR_7_15_23_31_AND_SEL_POS 24
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_IO2STMR_7_15_23_31_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_I2C_SDA_RD_IO2STMR_7_15_23_31_AND_SEL_DECL 24

/*** IO_CTRL_CDC_I2C_SDA_WR (0x50022008) ***/
/* This register is used for pad CDC_I2C_SDA_WR's IO function control.
Refer to */
/* the <a href="#io_pad_control_diagram">IO Function and
PAD Control</a> */
/* to understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_ADDR (0x50022008)
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_MASK_VAL 0x01c0401f
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_RMASK_VAL 0x01c0401f
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_WMASK_VAL 0x01c0401f
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad CDC_I2C_SDA_WR.
*   0x9: i2c4_data
*   0xa: uart0_rts_n
*   0xb: spi1_txd
*   0xc: spi0_cs3_n
*   0xd: spi1_cs3_n
*   0xe: spi2_cs3_n
*   0x10: fo_7
*   0x12: gpio_6
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_MUX_SEL_DECL (4:0)

/*
* Function i2c4_data can be selected through the following pads:
*   COMMC_3
*   COMMF_3
*   PORTA_DO
*   CDC_I2C_SDA_RD
*   CDC_I2C_SDA_WR
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C4_DATA_AND_SEL for pad CDC_I2C_SDA_WR, a
* separate receiver enable for the I2C4_DATA pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_I2C4_DATA_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_I2C4_DATA_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_I2C4_DATA_AND_SEL_POS 14
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_I2C4_DATA_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_I2C4_DATA_AND_SEL_DECL 14

/*
* Function fi_7 can be selected through the following pads:
*   COMMB_3
*   CDC_I2C_SDA_WR
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control FI_7_AND_SEL for pad CDC_I2C_SDA_WR, a
* separate receiver enable for the FI_7 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_FI_7_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_FI_7_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_FI_7_AND_SEL_POS 22
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_FI_7_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_FI_7_AND_SEL_DECL 22

/*
* This field is used to control GPIO_6_AND_SEL for pad CDC_I2C_SDA_WR, a
* separate receiver enable for the GPIO_6 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_GPIO_6_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_GPIO_6_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_GPIO_6_AND_SEL_POS 23
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_GPIO_6_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_GPIO_6_AND_SEL_DECL 23

/*
* Function io2stmr_6_14_22_30 can be selected through the following pads:
*   COMMB_2
*   PORTB_DI
*   PORTD_DI
*   CDC_I2C_SDA_WR
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_6_14_22_30_AND_SEL for pad
* CDC_I2C_SDA_WR, a separate receiver enable for the IO2STMR_6_14_22_30 pad
* function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_IO2STMR_6_14_22_30_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_IO2STMR_6_14_22_30_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_IO2STMR_6_14_22_30_AND_SEL_POS 24
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_IO2STMR_6_14_22_30_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_I2C_SDA_WR_IO2STMR_6_14_22_30_AND_SEL_DECL 24

/*** IO_CTRL_CDC_INT_N (0x5002200c) ***/
/* This register is used for pad CDC_INT_N's IO function control.Refer to the */
/* <a href="#io_pad_control_diagram">IO Function & PAD Controldiagram</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_CDC_INT_N_ADDR (0x5002200c)
#define IAXXX_IO_CTRL_CDC_INT_N_MASK_VAL 0x01c1c01f
#define IAXXX_IO_CTRL_CDC_INT_N_RMASK_VAL 0x01c1c01f
#define IAXXX_IO_CTRL_CDC_INT_N_WMASK_VAL 0x01c1c01f
#define IAXXX_IO_CTRL_CDC_INT_N_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad CDC_INT_N.
*   0x9: i2c3_clk
*   0xb: spi1_cs0_n
*   0xc: spi0_cs1_n
*   0xd: spi1_cs1_n
*   0xe: spi2_cs1_n
*   0x10: fo_8
*   0x12: gpio_4
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_CDC_INT_N_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_CDC_INT_N_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_CDC_INT_N_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_CDC_INT_N_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_CDC_INT_N_MUX_SEL_DECL (4:0)

/*
* Function i2c3_clk can be selected through the following pads:
*   COMMC_0
*   COMMF_0
*   PORTA_CLK
*   CDC_INT_N
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C3_CLK_AND_SEL for pad CDC_INT_N, a
* separate receiver enable for the I2C3_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_INT_N_I2C3_CLK_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_CDC_INT_N_I2C3_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_INT_N_I2C3_CLK_AND_SEL_POS 14
#define IAXXX_IO_CTRL_CDC_INT_N_I2C3_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_INT_N_I2C3_CLK_AND_SEL_DECL 14

/*
* Function uart1_sin can be selected through the following pads:
*   COMMB_2
*   COMMD_2
*   PORTD_CLK
*   PORTE_CLK
*   CDC_INT_N
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control UART1_SIN_AND_SEL for pad CDC_INT_N, a
* separate receiver enable for the UART1_SIN pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_INT_N_UART1_SIN_AND_SEL_MASK 0x00008000
#define IAXXX_IO_CTRL_CDC_INT_N_UART1_SIN_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_INT_N_UART1_SIN_AND_SEL_POS 15
#define IAXXX_IO_CTRL_CDC_INT_N_UART1_SIN_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_INT_N_UART1_SIN_AND_SEL_DECL 15

/*
* Function spi1_cs0_n can be selected through the following pads:
*   COMMB_2
*   PORTE_DI
*   CDC_INT_N
*   CDC_I2C_SCL
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SPI1_CS0_N_AND_SEL for pad CDC_INT_N, a
* separate receiver enable for the SPI1_CS0_N pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_INT_N_SPI1_CS0_N_AND_SEL_MASK 0x00010000
#define IAXXX_IO_CTRL_CDC_INT_N_SPI1_CS0_N_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_INT_N_SPI1_CS0_N_AND_SEL_POS 16
#define IAXXX_IO_CTRL_CDC_INT_N_SPI1_CS0_N_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_INT_N_SPI1_CS0_N_AND_SEL_DECL 16

/*
* Function fi_8 can be selected through the following pads:
*   PORTA_CLK
*   CDC_INT_N
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control FI_8_AND_SEL for pad CDC_INT_N, a separate
* receiver enable for the FI_8 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_INT_N_FI_8_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_CDC_INT_N_FI_8_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_INT_N_FI_8_AND_SEL_POS 22
#define IAXXX_IO_CTRL_CDC_INT_N_FI_8_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_INT_N_FI_8_AND_SEL_DECL 22

/*
* This field is used to control GPIO_4_AND_SEL for pad CDC_INT_N, a separate
* receiver enable for the GPIO_4 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_INT_N_GPIO_4_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_CDC_INT_N_GPIO_4_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_CDC_INT_N_GPIO_4_AND_SEL_POS 23
#define IAXXX_IO_CTRL_CDC_INT_N_GPIO_4_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_INT_N_GPIO_4_AND_SEL_DECL 23

/*
* Function io2stmr_4_12_20_28 can be selected through the following pads:
*   COMMB_0
*   PORTB_CLK
*   PORTD_CLK
*   CDC_INT_N
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_4_12_20_28_AND_SEL for pad
* CDC_INT_N, a separate receiver enable for the IO2STMR_4_12_20_28 pad
* function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_INT_N_IO2STMR_4_12_20_28_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_CDC_INT_N_IO2STMR_4_12_20_28_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_INT_N_IO2STMR_4_12_20_28_AND_SEL_POS 24
#define IAXXX_IO_CTRL_CDC_INT_N_IO2STMR_4_12_20_28_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_INT_N_IO2STMR_4_12_20_28_AND_SEL_DECL 24

/*** IO_CTRL_CDC_MCLK (0x50022010) ***/
/* This register is used for pad CDC_MCLK's IO function control.
Refer to the <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_CDC_MCLK_ADDR (0x50022010)
#define IAXXX_IO_CTRL_CDC_MCLK_MASK_VAL 0x0090201f
#define IAXXX_IO_CTRL_CDC_MCLK_RMASK_VAL 0x0090201f
#define IAXXX_IO_CTRL_CDC_MCLK_WMASK_VAL 0x0090201f
#define IAXXX_IO_CTRL_CDC_MCLK_RESET_VAL 0x0000001f

/*
* This field is used to select a function source for output data and output
* enable control for pad CDC_MCLK.
*   0x6: i2s_cdc_clk
*   0x12: gpio_56
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_CDC_MCLK_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_CDC_MCLK_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_CDC_MCLK_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_CDC_MCLK_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_CDC_MCLK_MUX_SEL_DECL (4:0)

/*
* This field is used to control CDC0_CLK_AND_SEL for pad CDC_MCLK, a
* separate receiver enable for the CDC0_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_MCLK_CDC0_CLK_AND_SEL_MASK 0x00002000
#define IAXXX_IO_CTRL_CDC_MCLK_CDC0_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_MCLK_CDC0_CLK_AND_SEL_POS 13
#define IAXXX_IO_CTRL_CDC_MCLK_CDC0_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_MCLK_CDC0_CLK_AND_SEL_DECL 13

/*
* This field is used to control CDC_CLK2CNR_AND_SEL for pad CDC_MCLK, a
* separate receiver enable for the CDC_CLK2CNR pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_MCLK_CDC_CLK2CNR_AND_SEL_MASK 0x00100000
#define IAXXX_IO_CTRL_CDC_MCLK_CDC_CLK2CNR_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_MCLK_CDC_CLK2CNR_AND_SEL_POS 20
#define IAXXX_IO_CTRL_CDC_MCLK_CDC_CLK2CNR_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_MCLK_CDC_CLK2CNR_AND_SEL_DECL 20

/*
* This field is used to control GPIO_56_AND_SEL for pad CDC_MCLK, a separate
* receiver enable for the GPIO_56 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_MCLK_GPIO_56_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_CDC_MCLK_GPIO_56_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_MCLK_GPIO_56_AND_SEL_POS 23
#define IAXXX_IO_CTRL_CDC_MCLK_GPIO_56_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_MCLK_GPIO_56_AND_SEL_DECL 23

/*** IO_CTRL_CDC_PDM0 (0x50022014) ***/
/* This register is used for pad CDC_PDM0's IO function control.
Refer to the <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_CDC_PDM0_ADDR (0x50022014)
#define IAXXX_IO_CTRL_CDC_PDM0_MASK_VAL 0x0090201f
#define IAXXX_IO_CTRL_CDC_PDM0_RMASK_VAL 0x0090201f
#define IAXXX_IO_CTRL_CDC_PDM0_WMASK_VAL 0x0090201f
#define IAXXX_IO_CTRL_CDC_PDM0_RESET_VAL 0x0000201f

/*
* This field is used to select a function source for output data and output
* enable control for pad CDC_PDM0.
*   0x12: gpio_57
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_CDC_PDM0_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_CDC_PDM0_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_CDC_PDM0_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_CDC_PDM0_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_CDC_PDM0_MUX_SEL_DECL (4:0)

/*
* This field is used to control CDC_ADC_0_AND_SEL for pad CDC_PDM0, a
* separate receiver enable for the CDC_ADC_0 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_PDM0_CDC_ADC_0_AND_SEL_MASK 0x00002000
#define IAXXX_IO_CTRL_CDC_PDM0_CDC_ADC_0_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_CDC_PDM0_CDC_ADC_0_AND_SEL_POS 13
#define IAXXX_IO_CTRL_CDC_PDM0_CDC_ADC_0_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_PDM0_CDC_ADC_0_AND_SEL_DECL 13

/*
* Function cal_clk2cnr can be selected through the following pads:
*   COMMA_0
*   COMMA_1
*   COMMB_1
*   CDC_PDM0
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control CAL_CLK2CNR_AND_SEL for pad CDC_PDM0, a
* separate receiver enable for the CAL_CLK2CNR pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_PDM0_CAL_CLK2CNR_AND_SEL_MASK 0x00100000
#define IAXXX_IO_CTRL_CDC_PDM0_CAL_CLK2CNR_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_PDM0_CAL_CLK2CNR_AND_SEL_POS 20
#define IAXXX_IO_CTRL_CDC_PDM0_CAL_CLK2CNR_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_PDM0_CAL_CLK2CNR_AND_SEL_DECL 20

/*
* This field is used to control GPIO_57_AND_SEL for pad CDC_PDM0, a separate
* receiver enable for the GPIO_57 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_PDM0_GPIO_57_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_CDC_PDM0_GPIO_57_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_PDM0_GPIO_57_AND_SEL_POS 23
#define IAXXX_IO_CTRL_CDC_PDM0_GPIO_57_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_PDM0_GPIO_57_AND_SEL_DECL 23

/*** IO_CTRL_CDC_PDM1 (0x50022018) ***/
/* This register is used for pad CDC_PDM1's IO function control.
Refer to the <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_CDC_PDM1_ADDR (0x50022018)
#define IAXXX_IO_CTRL_CDC_PDM1_MASK_VAL 0x0080201f
#define IAXXX_IO_CTRL_CDC_PDM1_RMASK_VAL 0x0080201f
#define IAXXX_IO_CTRL_CDC_PDM1_WMASK_VAL 0x0080201f
#define IAXXX_IO_CTRL_CDC_PDM1_RESET_VAL 0x0000201f

/*
* This field is used to select a function source for output data and output
* enable control for pad CDC_PDM1.
*   0x12: gpio_58
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_CDC_PDM1_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_CDC_PDM1_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_CDC_PDM1_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_CDC_PDM1_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_CDC_PDM1_MUX_SEL_DECL (4:0)

/*
* This field is used to control CDC_ADC_1_AND_SEL for pad CDC_PDM1, a
* separate receiver enable for the CDC_ADC_1 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_PDM1_CDC_ADC_1_AND_SEL_MASK 0x00002000
#define IAXXX_IO_CTRL_CDC_PDM1_CDC_ADC_1_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_CDC_PDM1_CDC_ADC_1_AND_SEL_POS 13
#define IAXXX_IO_CTRL_CDC_PDM1_CDC_ADC_1_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_PDM1_CDC_ADC_1_AND_SEL_DECL 13

/*
* This field is used to control GPIO_58_AND_SEL for pad CDC_PDM1, a separate
* receiver enable for the GPIO_58 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_PDM1_GPIO_58_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_CDC_PDM1_GPIO_58_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_PDM1_GPIO_58_AND_SEL_POS 23
#define IAXXX_IO_CTRL_CDC_PDM1_GPIO_58_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_PDM1_GPIO_58_AND_SEL_DECL 23

/*** IO_CTRL_CDC_PDM2 (0x5002201c) ***/
/* This register is used for pad CDC_PDM2's IO function controlRefer to the <a*/
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_CDC_PDM2_ADDR (0x5002201c)
#define IAXXX_IO_CTRL_CDC_PDM2_MASK_VAL 0x0080201f
#define IAXXX_IO_CTRL_CDC_PDM2_RMASK_VAL 0x0080201f
#define IAXXX_IO_CTRL_CDC_PDM2_WMASK_VAL 0x0080201f
#define IAXXX_IO_CTRL_CDC_PDM2_RESET_VAL 0x0000201f

/*
* This field is used to select a function source for output data and output
* enable control for pad CDC_PDM2.
*   0x12: gpio_59
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_CDC_PDM2_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_CDC_PDM2_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_CDC_PDM2_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_CDC_PDM2_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_CDC_PDM2_MUX_SEL_DECL (4:0)

/*
* This field is used to control CDC_ADC_2_AND_SEL for pad CDC_PDM2, a
* separate receiver enable for the CDC_ADC_2 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_PDM2_CDC_ADC_2_AND_SEL_MASK 0x00002000
#define IAXXX_IO_CTRL_CDC_PDM2_CDC_ADC_2_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_CDC_PDM2_CDC_ADC_2_AND_SEL_POS 13
#define IAXXX_IO_CTRL_CDC_PDM2_CDC_ADC_2_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_PDM2_CDC_ADC_2_AND_SEL_DECL 13

/*
* This field is used to control GPIO_59_AND_SEL for pad CDC_PDM2, a separate
* receiver enable for the GPIO_59 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_PDM2_GPIO_59_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_CDC_PDM2_GPIO_59_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_PDM2_GPIO_59_AND_SEL_POS 23
#define IAXXX_IO_CTRL_CDC_PDM2_GPIO_59_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_PDM2_GPIO_59_AND_SEL_DECL 23

/*** IO_CTRL_CDC_PWRON (0x50022020) ***/
/* This register is used for pad CDC_PWRON's IO function control.Refer to the*/
/* <a href="#io_pad_control_diagram">IO Function and PAD Control
diagram</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_CDC_PWRON_ADDR (0x50022020)
#define IAXXX_IO_CTRL_CDC_PWRON_MASK_VAL 0x01c1c01f
#define IAXXX_IO_CTRL_CDC_PWRON_RMASK_VAL 0x01c1c01f
#define IAXXX_IO_CTRL_CDC_PWRON_WMASK_VAL 0x01c1c01f
#define IAXXX_IO_CTRL_CDC_PWRON_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad CDC_PWRON.
*   0x9: i2c4_clk
*   0xc: spi0_cs1_n
*   0xd: spi1_cs1_n
*   0xe: spi2_cs1_n
*   0x10: fo_4
*   0x12: gpio_8
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_CDC_PWRON_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_CDC_PWRON_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_CDC_PWRON_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_CDC_PWRON_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_CDC_PWRON_MUX_SEL_DECL (4:0)

/*
* Function i2c4_clk can be selected through the following pads:
*   COMMC_2
*   COMMF_2
*   PORTA_DI
*   CDC_PWRON
*   CDC_I2C_SCL
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C4_CLK_AND_SEL for pad CDC_PWRON, a
* separate receiver enable for the I2C4_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_PWRON_I2C4_CLK_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_CDC_PWRON_I2C4_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_PWRON_I2C4_CLK_AND_SEL_POS 14
#define IAXXX_IO_CTRL_CDC_PWRON_I2C4_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_PWRON_I2C4_CLK_AND_SEL_DECL 14

/*
* Function uart0_sin can be selected through the following pads:
*   COMMA_2
*   COMMC_2
*   COMMD_0
*   PORTD_DI
*   PORTE_DI
*   CDC_PWRON
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control UART0_SIN_AND_SEL for pad CDC_PWRON, a
* separate receiver enable for the UART0_SIN pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_PWRON_UART0_SIN_AND_SEL_MASK 0x00008000
#define IAXXX_IO_CTRL_CDC_PWRON_UART0_SIN_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_PWRON_UART0_SIN_AND_SEL_POS 15
#define IAXXX_IO_CTRL_CDC_PWRON_UART0_SIN_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_PWRON_UART0_SIN_AND_SEL_DECL 15

/*
* Function spi1_rxd can be selected through the following pads:
*   COMMB_0
*   PORTE_CLK
*   CDC_PWRON
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SPI1_RXD_AND_SEL for pad CDC_PWRON, a
* separate receiver enable for the SPI1_RXD pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_PWRON_SPI1_RXD_AND_SEL_MASK 0x00010000
#define IAXXX_IO_CTRL_CDC_PWRON_SPI1_RXD_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_PWRON_SPI1_RXD_AND_SEL_POS 16
#define IAXXX_IO_CTRL_CDC_PWRON_SPI1_RXD_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_PWRON_SPI1_RXD_AND_SEL_DECL 16

/*
* Function fi_4 can be selected through the following pads:
*   COMMB_0
*   CDC_PWRON
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control FI_4_AND_SEL for pad CDC_PWRON, a separate
* receiver enable for the FI_4 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_PWRON_FI_4_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_CDC_PWRON_FI_4_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_PWRON_FI_4_AND_SEL_POS 22
#define IAXXX_IO_CTRL_CDC_PWRON_FI_4_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_PWRON_FI_4_AND_SEL_DECL 22

/*
* This field is used to control GPIO_8_AND_SEL for pad CDC_PWRON, a separate
* receiver enable for the GPIO_8 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_PWRON_GPIO_8_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_CDC_PWRON_GPIO_8_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_CDC_PWRON_GPIO_8_AND_SEL_POS 23
#define IAXXX_IO_CTRL_CDC_PWRON_GPIO_8_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_PWRON_GPIO_8_AND_SEL_DECL 23

/*
* Function io2stmr_0_8_16_24 can be selected through the following pads:
*   COMMA_2
*   PORTA_CLK
*   PORTC_CLK
*   PORTE_CLK
*   CDC_PWRON
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_0_8_16_24_AND_SEL for pad CDC_PWRON,
* a separate receiver enable for the IO2STMR_0_8_16_24 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_PWRON_IO2STMR_0_8_16_24_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_CDC_PWRON_IO2STMR_0_8_16_24_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_PWRON_IO2STMR_0_8_16_24_AND_SEL_POS 24
#define IAXXX_IO_CTRL_CDC_PWRON_IO2STMR_0_8_16_24_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_PWRON_IO2STMR_0_8_16_24_AND_SEL_DECL 24

/*** IO_CTRL_CDC_RESET_N (0x50022024) ***/
/* This register is used for pad CDC_RESET_N's IO function control.
Refer to the */
/* <a href="#io_pad_control_diagram">IO Function and PAD
Control diagram</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_CDC_RESET_N_ADDR (0x50022024)
#define IAXXX_IO_CTRL_CDC_RESET_N_MASK_VAL 0x01c0401f
#define IAXXX_IO_CTRL_CDC_RESET_N_RMASK_VAL 0x01c0401f
#define IAXXX_IO_CTRL_CDC_RESET_N_WMASK_VAL 0x01c0401f
#define IAXXX_IO_CTRL_CDC_RESET_N_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad CDC_RESET_N.
*   0x9: i2c3_data
*   0xa: uart1_sout
*   0xb: spi1_txd
*   0xc: spi0_cs3_n
*   0xd: spi1_cs3_n
*   0xe: spi2_cs3_n
*   0x10: fo_9
*   0x12: gpio_3
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_CDC_RESET_N_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_CDC_RESET_N_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_CDC_RESET_N_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_CDC_RESET_N_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_CDC_RESET_N_MUX_SEL_DECL (4:0)

/*
* Function i2c3_data can be selected through the following pads:
*   COMMC_1
*   COMMF_1
*   PORTA_FS
*   CDC_RESET_N
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C3_DATA_AND_SEL for pad CDC_RESET_N, a
* separate receiver enable for the I2C3_DATA pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_RESET_N_I2C3_DATA_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_CDC_RESET_N_I2C3_DATA_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_RESET_N_I2C3_DATA_AND_SEL_POS 14
#define IAXXX_IO_CTRL_CDC_RESET_N_I2C3_DATA_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_RESET_N_I2C3_DATA_AND_SEL_DECL 14

/*
* Function fi_9 can be selected through the following pads:
*   PORTA_FS
*   CDC_RESET_N
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control FI_9_AND_SEL for pad CDC_RESET_N, a separate
* receiver enable for the FI_9 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_RESET_N_FI_9_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_CDC_RESET_N_FI_9_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_RESET_N_FI_9_AND_SEL_POS 22
#define IAXXX_IO_CTRL_CDC_RESET_N_FI_9_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_RESET_N_FI_9_AND_SEL_DECL 22

/*
* This field is used to control GPIO_3_AND_SEL for pad CDC_RESET_N, a
* separate receiver enable for the GPIO_3 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_RESET_N_GPIO_3_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_CDC_RESET_N_GPIO_3_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_CDC_RESET_N_GPIO_3_AND_SEL_POS 23
#define IAXXX_IO_CTRL_CDC_RESET_N_GPIO_3_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_RESET_N_GPIO_3_AND_SEL_DECL 23

/*
* Function io2stmr_3_11_19_27 can be selected through the following pads:
*   COMMA_1
*   PORTA_DO
*   PORTC_DO
*   PORTE_DO
*   CDC_RESET_N
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_3_11_19_27_AND_SEL for pad
* CDC_RESET_N, a separate receiver enable for the IO2STMR_3_11_19_27 pad
* function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CDC_RESET_N_IO2STMR_3_11_19_27_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_CDC_RESET_N_IO2STMR_3_11_19_27_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CDC_RESET_N_IO2STMR_3_11_19_27_AND_SEL_POS 24
#define IAXXX_IO_CTRL_CDC_RESET_N_IO2STMR_3_11_19_27_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CDC_RESET_N_IO2STMR_3_11_19_27_AND_SEL_DECL 24

/*** IO_CTRL_CLK_IN (0x50022028) ***/
/* This register is used for pad CLK_IN's IO function control.Refer to the <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_CLK_IN_ADDR (0x50022028)
#define IAXXX_IO_CTRL_CLK_IN_MASK_VAL 0x0080005f
#define IAXXX_IO_CTRL_CLK_IN_RMASK_VAL 0x0080005f
#define IAXXX_IO_CTRL_CLK_IN_WMASK_VAL 0x0080005f
#define IAXXX_IO_CTRL_CLK_IN_RESET_VAL 0x0000005f

/*
* This field is used to select a function source for output data and output
* enable control for pad CLK_IN.
*   0x12: gpio_60
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_CLK_IN_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_CLK_IN_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_CLK_IN_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_CLK_IN_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_CLK_IN_MUX_SEL_DECL (4:0)

/*
* This field is used to control SYS_CLK_IN_AND_SEL for pad CLK_IN, a
* separate receiver enable for the SYS_CLK_IN pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CLK_IN_SYS_CLK_IN_AND_SEL_MASK 0x00000040
#define IAXXX_IO_CTRL_CLK_IN_SYS_CLK_IN_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_CLK_IN_SYS_CLK_IN_AND_SEL_POS 6
#define IAXXX_IO_CTRL_CLK_IN_SYS_CLK_IN_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CLK_IN_SYS_CLK_IN_AND_SEL_DECL 6

/*
* This field is used to control GPIO_60_AND_SEL for pad CLK_IN, a separate
* receiver enable for the GPIO_60 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_CLK_IN_GPIO_60_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_CLK_IN_GPIO_60_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_CLK_IN_GPIO_60_AND_SEL_POS 23
#define IAXXX_IO_CTRL_CLK_IN_GPIO_60_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_CLK_IN_GPIO_60_AND_SEL_DECL 23

/*** IO_CTRL_COMMA_0 (0x5002202c) ***/
/* This register is used for pad COMMA_0's IO function control.Refer to the <a*/
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_COMMA_0_ADDR (0x5002202c)
#define IAXXX_IO_CTRL_COMMA_0_MASK_VAL 0x01d1c01f
#define IAXXX_IO_CTRL_COMMA_0_RMASK_VAL 0x01d1c01f
#define IAXXX_IO_CTRL_COMMA_0_WMASK_VAL 0x01d1c01f
#define IAXXX_IO_CTRL_COMMA_0_RESET_VAL 0x0180001f

/*
* This field is used to select a function source for output data and output
* enable control for pad COMMA_0.
*   0x9: i2c0_clk
*   0xd: spi1_cs1_n
*   0xe: spi2_cs1_n
*   0x12: gpio_10
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_COMMA_0_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_COMMA_0_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_COMMA_0_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_COMMA_0_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_COMMA_0_MUX_SEL_DECL (4:0)

/*
* Function i2c0_clk can be selected through the following pads:
*   COMMA_0
*   COMMD_0
*   PORTD_CLK
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C0_CLK_AND_SEL for pad COMMA_0, a separate
* receiver enable for the I2C0_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMA_0_I2C0_CLK_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_COMMA_0_I2C0_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMA_0_I2C0_CLK_AND_SEL_POS 14
#define IAXXX_IO_CTRL_COMMA_0_I2C0_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMA_0_I2C0_CLK_AND_SEL_DECL 14

/*
* Function uart0_cts_n can be selected through the following pads:
*   COMMA_0
*   COMMC_0
*   CDC_I2C_SCL
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control UART0_CTS_N_AND_SEL for pad COMMA_0, a
* separate receiver enable for the UART0_CTS_N pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMA_0_UART0_CTS_N_AND_SEL_MASK 0x00008000
#define IAXXX_IO_CTRL_COMMA_0_UART0_CTS_N_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMA_0_UART0_CTS_N_AND_SEL_POS 15
#define IAXXX_IO_CTRL_COMMA_0_UART0_CTS_N_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMA_0_UART0_CTS_N_AND_SEL_DECL 15

/*
* Function spi0_rxd can be selected through the following pads:
*   COMMA_0
*   COMMD_0
*   PORTD_CLK
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SPI0_RXD_AND_SEL for pad COMMA_0, a separate
* receiver enable for the SPI0_RXD pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMA_0_SPI0_RXD_AND_SEL_MASK 0x00010000
#define IAXXX_IO_CTRL_COMMA_0_SPI0_RXD_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMA_0_SPI0_RXD_AND_SEL_POS 16
#define IAXXX_IO_CTRL_COMMA_0_SPI0_RXD_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMA_0_SPI0_RXD_AND_SEL_DECL 16

/*
* Function cal_clk2cnr can be selected through the following pads:
*   COMMA_0
*   COMMA_1
*   COMMB_1
*   CDC_PDM0
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control CAL_CLK2CNR_AND_SEL for pad COMMA_0, a
* separate receiver enable for the CAL_CLK2CNR pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMA_0_CAL_CLK2CNR_AND_SEL_MASK 0x00100000
#define IAXXX_IO_CTRL_COMMA_0_CAL_CLK2CNR_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMA_0_CAL_CLK2CNR_AND_SEL_POS 20
#define IAXXX_IO_CTRL_COMMA_0_CAL_CLK2CNR_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMA_0_CAL_CLK2CNR_AND_SEL_DECL 20

/*
* This field is used to control FI_0_AND_SEL for pad COMMA_0, a separate
* receiver enable for the FI_0 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMA_0_FI_0_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_COMMA_0_FI_0_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMA_0_FI_0_AND_SEL_POS 22
#define IAXXX_IO_CTRL_COMMA_0_FI_0_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMA_0_FI_0_AND_SEL_DECL 22

/*
* This field is used to control GPIO_10_AND_SEL for pad COMMA_0, a separate
* receiver enable for the GPIO_10 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMA_0_GPIO_10_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_COMMA_0_GPIO_10_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMMA_0_GPIO_10_AND_SEL_POS 23
#define IAXXX_IO_CTRL_COMMA_0_GPIO_10_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMA_0_GPIO_10_AND_SEL_DECL 23

/*
* Function io2stmr_2_10_18_26 can be selected through the following pads:
*   COMMA_0
*   INTR_EVNT
*   PORTA_DI
*   PORTC_DI
*   PORTE_DI
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_2_10_18_26_AND_SEL for pad COMMA_0,
* a separate receiver enable for the IO2STMR_2_10_18_26 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMA_0_IO2STMR_2_10_18_26_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_COMMA_0_IO2STMR_2_10_18_26_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMMA_0_IO2STMR_2_10_18_26_AND_SEL_POS 24
#define IAXXX_IO_CTRL_COMMA_0_IO2STMR_2_10_18_26_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMA_0_IO2STMR_2_10_18_26_AND_SEL_DECL 24

/*** IO_CTRL_COMMA_1 (0x50022030) ***/
/* This register is used for pad COMMA_1's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_COMMA_1_ADDR (0x50022030)
#define IAXXX_IO_CTRL_COMMA_1_MASK_VAL 0x01d1401f
#define IAXXX_IO_CTRL_COMMA_1_RMASK_VAL 0x01d1401f
#define IAXXX_IO_CTRL_COMMA_1_WMASK_VAL 0x01d1401f
#define IAXXX_IO_CTRL_COMMA_1_RESET_VAL 0x0180001f

/*
* This field is used to select a function source for output data and output
* enable control for pad COMMA_1.
*   0x9: i2c0_data
*   0xa: uart0_rts_n
*   0xb: spi0_clk
*   0xd: spi1_cs2_n
*   0xe: spi2_cs2_n
*   0x12: gpio_11
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_COMMA_1_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_COMMA_1_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_COMMA_1_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_COMMA_1_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_COMMA_1_MUX_SEL_DECL (4:0)

/*
* Function i2c0_data can be selected through the following pads:
*   COMMA_1
*   COMMD_1
*   PORTD_FS
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C0_DATA_AND_SEL for pad COMMA_1, a
* separate receiver enable for the I2C0_DATA pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMA_1_I2C0_DATA_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_COMMA_1_I2C0_DATA_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMA_1_I2C0_DATA_AND_SEL_POS 14
#define IAXXX_IO_CTRL_COMMA_1_I2C0_DATA_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMA_1_I2C0_DATA_AND_SEL_DECL 14

/*
* Function spi0_clk can be selected through the following pads:
*   COMMA_1
*   COMMD_1
*   PORTD_FS
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SPI0_CLK_AND_SEL for pad COMMA_1, a separate
* receiver enable for the SPI0_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMA_1_SPI0_CLK_AND_SEL_MASK 0x00010000
#define IAXXX_IO_CTRL_COMMA_1_SPI0_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMA_1_SPI0_CLK_AND_SEL_POS 16
#define IAXXX_IO_CTRL_COMMA_1_SPI0_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMA_1_SPI0_CLK_AND_SEL_DECL 16

/*
* Function cal_clk2cnr can be selected through the following pads:
*   COMMA_0
*   COMMA_1
*   COMMB_1
*   CDC_PDM0
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control CAL_CLK2CNR_AND_SEL for pad COMMA_1, a
* separate receiver enable for the CAL_CLK2CNR pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMA_1_CAL_CLK2CNR_AND_SEL_MASK 0x00100000
#define IAXXX_IO_CTRL_COMMA_1_CAL_CLK2CNR_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMA_1_CAL_CLK2CNR_AND_SEL_POS 20
#define IAXXX_IO_CTRL_COMMA_1_CAL_CLK2CNR_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMA_1_CAL_CLK2CNR_AND_SEL_DECL 20

/*
* This field is used to control FI_1_AND_SEL for pad COMMA_1, a separate
* receiver enable for the FI_1 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMA_1_FI_1_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_COMMA_1_FI_1_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMA_1_FI_1_AND_SEL_POS 22
#define IAXXX_IO_CTRL_COMMA_1_FI_1_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMA_1_FI_1_AND_SEL_DECL 22

/*
* This field is used to control GPIO_11_AND_SEL for pad COMMA_1, a separate
* receiver enable for the GPIO_11 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMA_1_GPIO_11_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_COMMA_1_GPIO_11_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMMA_1_GPIO_11_AND_SEL_POS 23
#define IAXXX_IO_CTRL_COMMA_1_GPIO_11_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMA_1_GPIO_11_AND_SEL_DECL 23

/*
* Function io2stmr_3_11_19_27 can be selected through the following pads:
*   COMMA_1
*   PORTA_DO
*   PORTC_DO
*   PORTE_DO
*   CDC_RESET_N
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_3_11_19_27_AND_SEL for pad COMMA_1,
* a separate receiver enable for the IO2STMR_3_11_19_27 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMA_1_IO2STMR_3_11_19_27_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_COMMA_1_IO2STMR_3_11_19_27_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMMA_1_IO2STMR_3_11_19_27_AND_SEL_POS 24
#define IAXXX_IO_CTRL_COMMA_1_IO2STMR_3_11_19_27_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMA_1_IO2STMR_3_11_19_27_AND_SEL_DECL 24

/*** IO_CTRL_COMMA_2 (0x50022034) ***/
/* This register is used for pad COMMA_2's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_COMMA_2_ADDR (0x50022034)
#define IAXXX_IO_CTRL_COMMA_2_MASK_VAL 0x01c1801f
#define IAXXX_IO_CTRL_COMMA_2_RMASK_VAL 0x01c1801f
#define IAXXX_IO_CTRL_COMMA_2_WMASK_VAL 0x01c1801f
#define IAXXX_IO_CTRL_COMMA_2_RESET_VAL 0x0180001f

/*
* This field is used to select a function source for output data and output
* enable control for pad COMMA_2.
*   0xb: spi0_cs0_n
*   0xd: spi1_cs1_n
*   0xe: spi2_cs1_n
*   0x12: gpio_0
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_COMMA_2_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_COMMA_2_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_COMMA_2_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_COMMA_2_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_COMMA_2_MUX_SEL_DECL (4:0)

/*
* Function uart0_sin can be selected through the following pads:
*   COMMA_2
*   COMMC_2
*   COMMD_0
*   PORTD_DI
*   PORTE_DI
*   CDC_PWRON
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control UART0_SIN_AND_SEL for pad COMMA_2, a
* separate receiver enable for the UART0_SIN pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMA_2_UART0_SIN_AND_SEL_MASK 0x00008000
#define IAXXX_IO_CTRL_COMMA_2_UART0_SIN_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMA_2_UART0_SIN_AND_SEL_POS 15
#define IAXXX_IO_CTRL_COMMA_2_UART0_SIN_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMA_2_UART0_SIN_AND_SEL_DECL 15

/*
* Function spi0_cs0_n can be selected through the following pads:
*   COMMA_2
*   COMMD_2
*   PORTD_DI
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SPI0_CS0_N_AND_SEL for pad COMMA_2, a
* separate receiver enable for the SPI0_CS0_N pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMA_2_SPI0_CS0_N_AND_SEL_MASK 0x00010000
#define IAXXX_IO_CTRL_COMMA_2_SPI0_CS0_N_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMA_2_SPI0_CS0_N_AND_SEL_POS 16
#define IAXXX_IO_CTRL_COMMA_2_SPI0_CS0_N_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMA_2_SPI0_CS0_N_AND_SEL_DECL 16

/*
* This field is used to control FI_2_AND_SEL for pad COMMA_2, a separate
* receiver enable for the FI_2 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMA_2_FI_2_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_COMMA_2_FI_2_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMA_2_FI_2_AND_SEL_POS 22
#define IAXXX_IO_CTRL_COMMA_2_FI_2_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMA_2_FI_2_AND_SEL_DECL 22

/*
* This field is used to control GPIO_0_AND_SEL for pad COMMA_2, a separate
* receiver enable for the GPIO_0 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMA_2_GPIO_0_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_COMMA_2_GPIO_0_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMMA_2_GPIO_0_AND_SEL_POS 23
#define IAXXX_IO_CTRL_COMMA_2_GPIO_0_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMA_2_GPIO_0_AND_SEL_DECL 23

/*
* Function io2stmr_0_8_16_24 can be selected through the following pads:
*   COMMA_2
*   PORTA_CLK
*   PORTC_CLK
*   PORTE_CLK
*   CDC_PWRON
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_0_8_16_24_AND_SEL for pad COMMA_2, a
* separate receiver enable for the IO2STMR_0_8_16_24 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMA_2_IO2STMR_0_8_16_24_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_COMMA_2_IO2STMR_0_8_16_24_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMMA_2_IO2STMR_0_8_16_24_AND_SEL_POS 24
#define IAXXX_IO_CTRL_COMMA_2_IO2STMR_0_8_16_24_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMA_2_IO2STMR_0_8_16_24_AND_SEL_DECL 24

/*** IO_CTRL_COMMA_3 (0x50022038) ***/
/* This register is used for pad COMMA_3's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_COMMA_3_ADDR (0x50022038)
#define IAXXX_IO_CTRL_COMMA_3_MASK_VAL 0x01c0001f
#define IAXXX_IO_CTRL_COMMA_3_RMASK_VAL 0x01c0001f
#define IAXXX_IO_CTRL_COMMA_3_WMASK_VAL 0x01c0001f
#define IAXXX_IO_CTRL_COMMA_3_RESET_VAL 0x0180001f

/*
* This field is used to select a function source for output data and output
* enable control for pad COMMA_3.
*   0xa: uart0_sout
*   0xb: spi0_txd
*   0xd: spi1_cs3_n
*   0xe: spi2_cs3_n
*   0x12: gpio_1
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_COMMA_3_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_COMMA_3_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_COMMA_3_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_COMMA_3_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_COMMA_3_MUX_SEL_DECL (4:0)

/*
* This field is used to control FI_3_AND_SEL for pad COMMA_3, a separate
* receiver enable for the FI_3 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMA_3_FI_3_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_COMMA_3_FI_3_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMA_3_FI_3_AND_SEL_POS 22
#define IAXXX_IO_CTRL_COMMA_3_FI_3_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMA_3_FI_3_AND_SEL_DECL 22

/*
* This field is used to control GPIO_1_AND_SEL for pad COMMA_3, a separate
* receiver enable for the GPIO_1 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMA_3_GPIO_1_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_COMMA_3_GPIO_1_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMMA_3_GPIO_1_AND_SEL_POS 23
#define IAXXX_IO_CTRL_COMMA_3_GPIO_1_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMA_3_GPIO_1_AND_SEL_DECL 23

/*
* Function io2stmr_1_9_17_25 can be selected through the following pads:
*   COMMA_3
*   INTR_API
*   PORTA_FS
*   PORTC_FS
*   PORTE_FS
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_1_9_17_25_AND_SEL for pad COMMA_3, a
* separate receiver enable for the IO2STMR_1_9_17_25 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMA_3_IO2STMR_1_9_17_25_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_COMMA_3_IO2STMR_1_9_17_25_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMMA_3_IO2STMR_1_9_17_25_AND_SEL_POS 24
#define IAXXX_IO_CTRL_COMMA_3_IO2STMR_1_9_17_25_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMA_3_IO2STMR_1_9_17_25_AND_SEL_DECL 24

/*** IO_CTRL_COMMB_0 (0x5002203c) ***/
/* This register is used for pad COMMB_0's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_COMMB_0_ADDR (0x5002203c)
#define IAXXX_IO_CTRL_COMMB_0_MASK_VAL 0x01d1c11f
#define IAXXX_IO_CTRL_COMMB_0_RMASK_VAL 0x01d1c11f
#define IAXXX_IO_CTRL_COMMB_0_WMASK_VAL 0x01d1c11f
#define IAXXX_IO_CTRL_COMMB_0_RESET_VAL 0x0180001f

/*
* This field is used to select a function source for output data and output
* enable control for pad COMMB_0.
*   0x5: i2s_pcm5_bclk
*   0x6: i2s_cdc_clk
*   0x9: i2c1_clk
*   0xc: spi0_cs1_n
*   0xe: spi2_cs1_n
*   0x10: fo_4
*   0x12: gpio_12
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_COMMB_0_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_COMMB_0_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_COMMB_0_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_COMMB_0_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_COMMB_0_MUX_SEL_DECL (4:0)

/*
* This field is used to control PCM5_BCLK_AND_SEL for pad COMMB_0, a
* separate receiver enable for the PCM5_BCLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMB_0_PCM5_BCLK_AND_SEL_MASK 0x00000100
#define IAXXX_IO_CTRL_COMMB_0_PCM5_BCLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMB_0_PCM5_BCLK_AND_SEL_POS 8
#define IAXXX_IO_CTRL_COMMB_0_PCM5_BCLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMB_0_PCM5_BCLK_AND_SEL_DECL 8

/*
* Function i2c1_clk can be selected through the following pads:
*   COMMB_0
*   COMME_0
*   PORTE_CLK
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C1_CLK_AND_SEL for pad COMMB_0, a separate
* receiver enable for the I2C1_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMB_0_I2C1_CLK_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_COMMB_0_I2C1_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMB_0_I2C1_CLK_AND_SEL_POS 14
#define IAXXX_IO_CTRL_COMMB_0_I2C1_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMB_0_I2C1_CLK_AND_SEL_DECL 14

/*
* This field is used to control UART1_CTS_N_AND_SEL for pad COMMB_0, a
* separate receiver enable for the UART1_CTS_N pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMB_0_UART1_CTS_N_AND_SEL_MASK 0x00008000
#define IAXXX_IO_CTRL_COMMB_0_UART1_CTS_N_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMB_0_UART1_CTS_N_AND_SEL_POS 15
#define IAXXX_IO_CTRL_COMMB_0_UART1_CTS_N_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMB_0_UART1_CTS_N_AND_SEL_DECL 15

/*
* Function spi1_rxd can be selected through the following pads:
*   COMMB_0
*   PORTE_CLK
*   CDC_PWRON
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SPI1_RXD_AND_SEL for pad COMMB_0, a separate
* receiver enable for the SPI1_RXD pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMB_0_SPI1_RXD_AND_SEL_MASK 0x00010000
#define IAXXX_IO_CTRL_COMMB_0_SPI1_RXD_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMB_0_SPI1_RXD_AND_SEL_POS 16
#define IAXXX_IO_CTRL_COMMB_0_SPI1_RXD_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMB_0_SPI1_RXD_AND_SEL_DECL 16

/*
* This field is used to control PORTF_CLK2CNR_AND_SEL for pad COMMB_0, a
* separate receiver enable for the PORTF_CLK2CNR pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMB_0_PORTF_CLK2CNR_AND_SEL_MASK 0x00100000
#define IAXXX_IO_CTRL_COMMB_0_PORTF_CLK2CNR_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMB_0_PORTF_CLK2CNR_AND_SEL_POS 20
#define IAXXX_IO_CTRL_COMMB_0_PORTF_CLK2CNR_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMB_0_PORTF_CLK2CNR_AND_SEL_DECL 20

/*
* Function fi_4 can be selected through the following pads:
*   COMMB_0
*   CDC_PWRON
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control FI_4_AND_SEL for pad COMMB_0, a separate
* receiver enable for the FI_4 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMB_0_FI_4_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_COMMB_0_FI_4_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMB_0_FI_4_AND_SEL_POS 22
#define IAXXX_IO_CTRL_COMMB_0_FI_4_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMB_0_FI_4_AND_SEL_DECL 22

/*
* This field is used to control GPIO_12_AND_SEL for pad COMMB_0, a separate
* receiver enable for the GPIO_12 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMB_0_GPIO_12_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_COMMB_0_GPIO_12_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMMB_0_GPIO_12_AND_SEL_POS 23
#define IAXXX_IO_CTRL_COMMB_0_GPIO_12_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMB_0_GPIO_12_AND_SEL_DECL 23

/*
* Function io2stmr_4_12_20_28 can be selected through the following pads:
*   COMMB_0
*   PORTB_CLK
*   PORTD_CLK
*   CDC_INT_N
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_4_12_20_28_AND_SEL for pad COMMB_0,
* a separate receiver enable for the IO2STMR_4_12_20_28 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMB_0_IO2STMR_4_12_20_28_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_COMMB_0_IO2STMR_4_12_20_28_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMMB_0_IO2STMR_4_12_20_28_AND_SEL_POS 24
#define IAXXX_IO_CTRL_COMMB_0_IO2STMR_4_12_20_28_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMB_0_IO2STMR_4_12_20_28_AND_SEL_DECL 24

/*** IO_CTRL_COMMB_1 (0x50022040) ***/
/* This register is used for pad COMMB_1's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_COMMB_1_ADDR (0x50022040)
#define IAXXX_IO_CTRL_COMMB_1_MASK_VAL 0x01d1411f
#define IAXXX_IO_CTRL_COMMB_1_RMASK_VAL 0x01d1411f
#define IAXXX_IO_CTRL_COMMB_1_WMASK_VAL 0x01d1411f
#define IAXXX_IO_CTRL_COMMB_1_RESET_VAL 0x0180001f

/*
* This field is used to select a function source for output data and output
* enable control for pad COMMB_1.
*   0x5: i2s_pcm5_fs
*   0x9: i2c1_data
*   0xa: uart1_rts_n
*   0xb: spi1_clk
*   0xc: spi0_cs2_n
*   0xe: spi2_cs2_n
*   0x10: fo_5
*   0x12: gpio_13
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_COMMB_1_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_COMMB_1_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_COMMB_1_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_COMMB_1_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_COMMB_1_MUX_SEL_DECL (4:0)

/*
* This field is used to control PCM5_FS_AND_SEL for pad COMMB_1, a separate
* receiver enable for the PCM5_FS pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMB_1_PCM5_FS_AND_SEL_MASK 0x00000100
#define IAXXX_IO_CTRL_COMMB_1_PCM5_FS_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMB_1_PCM5_FS_AND_SEL_POS 8
#define IAXXX_IO_CTRL_COMMB_1_PCM5_FS_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMB_1_PCM5_FS_AND_SEL_DECL 8

/*
* Function i2c1_data can be selected through the following pads:
*   COMMB_1
*   COMME_1
*   PORTE_FS
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C1_DATA_AND_SEL for pad COMMB_1, a
* separate receiver enable for the I2C1_DATA pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMB_1_I2C1_DATA_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_COMMB_1_I2C1_DATA_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMB_1_I2C1_DATA_AND_SEL_POS 14
#define IAXXX_IO_CTRL_COMMB_1_I2C1_DATA_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMB_1_I2C1_DATA_AND_SEL_DECL 14

/*
* Function spi1_clk can be selected through the following pads:
*   COMMB_1
*   PORTE_FS
*   CDC_I2C_SDA_RD
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SPI1_CLK_AND_SEL for pad COMMB_1, a separate
* receiver enable for the SPI1_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMB_1_SPI1_CLK_AND_SEL_MASK 0x00010000
#define IAXXX_IO_CTRL_COMMB_1_SPI1_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMB_1_SPI1_CLK_AND_SEL_POS 16
#define IAXXX_IO_CTRL_COMMB_1_SPI1_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMB_1_SPI1_CLK_AND_SEL_DECL 16

/*
* Function cal_clk2cnr can be selected through the following pads:
*   COMMA_0
*   COMMA_1
*   COMMB_1
*   CDC_PDM0
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control CAL_CLK2CNR_AND_SEL for pad COMMB_1, a
* separate receiver enable for the CAL_CLK2CNR pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMB_1_CAL_CLK2CNR_AND_SEL_MASK 0x00100000
#define IAXXX_IO_CTRL_COMMB_1_CAL_CLK2CNR_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMB_1_CAL_CLK2CNR_AND_SEL_POS 20
#define IAXXX_IO_CTRL_COMMB_1_CAL_CLK2CNR_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMB_1_CAL_CLK2CNR_AND_SEL_DECL 20

/*
* Function fi_5 can be selected through the following pads:
*   COMMB_1
*   CDC_I2C_SDA_RD
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control FI_5_AND_SEL for pad COMMB_1, a separate
* receiver enable for the FI_5 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMB_1_FI_5_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_COMMB_1_FI_5_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMB_1_FI_5_AND_SEL_POS 22
#define IAXXX_IO_CTRL_COMMB_1_FI_5_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMB_1_FI_5_AND_SEL_DECL 22

/*
* This field is used to control GPIO_13_AND_SEL for pad COMMB_1, a separate
* receiver enable for the GPIO_13 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMB_1_GPIO_13_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_COMMB_1_GPIO_13_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMMB_1_GPIO_13_AND_SEL_POS 23
#define IAXXX_IO_CTRL_COMMB_1_GPIO_13_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMB_1_GPIO_13_AND_SEL_DECL 23

/*
* Function io2stmr_5_13_21_29 can be selected through the following pads:
*   COMMB_1
*   PORTB_FS
*   PORTD_FS
*   CDC_I2C_SCL
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_5_13_21_29_AND_SEL for pad COMMB_1,
* a separate receiver enable for the IO2STMR_5_13_21_29 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMB_1_IO2STMR_5_13_21_29_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_COMMB_1_IO2STMR_5_13_21_29_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMMB_1_IO2STMR_5_13_21_29_AND_SEL_POS 24
#define IAXXX_IO_CTRL_COMMB_1_IO2STMR_5_13_21_29_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMB_1_IO2STMR_5_13_21_29_AND_SEL_DECL 24

/*** IO_CTRL_COMMB_2 (0x50022044) ***/
/* This register is used for pad COMMB_2's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_COMMB_2_ADDR (0x50022044)
#define IAXXX_IO_CTRL_COMMB_2_MASK_VAL 0x01c1c11f
#define IAXXX_IO_CTRL_COMMB_2_RMASK_VAL 0x01c1c11f
#define IAXXX_IO_CTRL_COMMB_2_WMASK_VAL 0x01c1c11f
#define IAXXX_IO_CTRL_COMMB_2_RESET_VAL 0x0180001f

/*
* This field is used to select a function source for output data and output
* enable control for pad COMMB_2.
*   0x9: i2c2_clk
*   0xb: spi1_cs0_n
*   0xc: spi0_cs1_n
*   0xe: spi2_cs1_n
*   0x10: fo_6
*   0x12: gpio_14
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_COMMB_2_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_COMMB_2_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_COMMB_2_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_COMMB_2_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_COMMB_2_MUX_SEL_DECL (4:0)

/*
* This field is used to control PCM5_DR_AND_SEL for pad COMMB_2, a separate
* receiver enable for the PCM5_DR pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMB_2_PCM5_DR_AND_SEL_MASK 0x00000100
#define IAXXX_IO_CTRL_COMMB_2_PCM5_DR_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMB_2_PCM5_DR_AND_SEL_POS 8
#define IAXXX_IO_CTRL_COMMB_2_PCM5_DR_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMB_2_PCM5_DR_AND_SEL_DECL 8

/*
* Function i2c2_clk can be selected through the following pads:
*   COMMB_2
*   COMME_2
*   PORTE_DI
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C2_CLK_AND_SEL for pad COMMB_2, a separate
* receiver enable for the I2C2_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMB_2_I2C2_CLK_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_COMMB_2_I2C2_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMB_2_I2C2_CLK_AND_SEL_POS 14
#define IAXXX_IO_CTRL_COMMB_2_I2C2_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMB_2_I2C2_CLK_AND_SEL_DECL 14

/*
* Function uart1_sin can be selected through the following pads:
*   COMMB_2
*   COMMD_2
*   PORTD_CLK
*   PORTE_CLK
*   CDC_INT_N
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control UART1_SIN_AND_SEL for pad COMMB_2, a
* separate receiver enable for the UART1_SIN pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMB_2_UART1_SIN_AND_SEL_MASK 0x00008000
#define IAXXX_IO_CTRL_COMMB_2_UART1_SIN_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMB_2_UART1_SIN_AND_SEL_POS 15
#define IAXXX_IO_CTRL_COMMB_2_UART1_SIN_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMB_2_UART1_SIN_AND_SEL_DECL 15

/*
* Function spi1_cs0_n can be selected through the following pads:
*   COMMB_2
*   PORTE_DI
*   CDC_INT_N
*   CDC_I2C_SCL
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SPI1_CS0_N_AND_SEL for pad COMMB_2, a
* separate receiver enable for the SPI1_CS0_N pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMB_2_SPI1_CS0_N_AND_SEL_MASK 0x00010000
#define IAXXX_IO_CTRL_COMMB_2_SPI1_CS0_N_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMB_2_SPI1_CS0_N_AND_SEL_POS 16
#define IAXXX_IO_CTRL_COMMB_2_SPI1_CS0_N_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMB_2_SPI1_CS0_N_AND_SEL_DECL 16

/*
* Function fi_6 can be selected through the following pads:
*   COMMB_2
*   CDC_I2C_SCL
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control FI_6_AND_SEL for pad COMMB_2, a separate
* receiver enable for the FI_6 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMB_2_FI_6_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_COMMB_2_FI_6_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMB_2_FI_6_AND_SEL_POS 22
#define IAXXX_IO_CTRL_COMMB_2_FI_6_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMB_2_FI_6_AND_SEL_DECL 22

/*
* This field is used to control GPIO_14_AND_SEL for pad COMMB_2, a separate
* receiver enable for the GPIO_14 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMB_2_GPIO_14_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_COMMB_2_GPIO_14_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMMB_2_GPIO_14_AND_SEL_POS 23
#define IAXXX_IO_CTRL_COMMB_2_GPIO_14_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMB_2_GPIO_14_AND_SEL_DECL 23

/*
* Function io2stmr_6_14_22_30 can be selected through the following pads:
*   COMMB_2
*   PORTB_DI
*   PORTD_DI
*   CDC_I2C_SDA_WR
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_6_14_22_30_AND_SEL for pad COMMB_2,
* a separate receiver enable for the IO2STMR_6_14_22_30 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMB_2_IO2STMR_6_14_22_30_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_COMMB_2_IO2STMR_6_14_22_30_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMMB_2_IO2STMR_6_14_22_30_AND_SEL_POS 24
#define IAXXX_IO_CTRL_COMMB_2_IO2STMR_6_14_22_30_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMB_2_IO2STMR_6_14_22_30_AND_SEL_DECL 24

/*** IO_CTRL_COMMB_3 (0x50022048) ***/
/* This register is used for pad COMMB_3's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_COMMB_3_ADDR (0x50022048)
#define IAXXX_IO_CTRL_COMMB_3_MASK_VAL 0x01c0401f
#define IAXXX_IO_CTRL_COMMB_3_RMASK_VAL 0x01c0401f
#define IAXXX_IO_CTRL_COMMB_3_WMASK_VAL 0x01c0401f
#define IAXXX_IO_CTRL_COMMB_3_RESET_VAL 0x0180001f

/*
* This field is used to select a function source for output data and output
* enable control for pad COMMB_3.
*   0x3: pcm5_dt
*   0x9: i2c2_data
*   0xa: uart1_sout
*   0xb: spi1_txd
*   0xc: spi0_cs3_n
*   0xe: spi2_cs3_n
*   0x10: fo_7
*   0x12: gpio_15
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_COMMB_3_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_COMMB_3_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_COMMB_3_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_COMMB_3_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_COMMB_3_MUX_SEL_DECL (4:0)

/*
* Function i2c2_data can be selected through the following pads:
*   COMMB_3
*   COMME_3
*   PORTE_DO
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C2_DATA_AND_SEL for pad COMMB_3, a
* separate receiver enable for the I2C2_DATA pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMB_3_I2C2_DATA_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_COMMB_3_I2C2_DATA_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMB_3_I2C2_DATA_AND_SEL_POS 14
#define IAXXX_IO_CTRL_COMMB_3_I2C2_DATA_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMB_3_I2C2_DATA_AND_SEL_DECL 14

/*
* Function fi_7 can be selected through the following pads:
*   COMMB_3
*   CDC_I2C_SDA_WR
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control FI_7_AND_SEL for pad COMMB_3, a separate
* receiver enable for the FI_7 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMB_3_FI_7_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_COMMB_3_FI_7_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMB_3_FI_7_AND_SEL_POS 22
#define IAXXX_IO_CTRL_COMMB_3_FI_7_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMB_3_FI_7_AND_SEL_DECL 22

/*
* This field is used to control GPIO_15_AND_SEL for pad COMMB_3, a separate
* receiver enable for the GPIO_15 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMB_3_GPIO_15_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_COMMB_3_GPIO_15_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMMB_3_GPIO_15_AND_SEL_POS 23
#define IAXXX_IO_CTRL_COMMB_3_GPIO_15_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMB_3_GPIO_15_AND_SEL_DECL 23

/*
* Function io2stmr_7_15_23_31 can be selected through the following pads:
*   COMMB_3
*   PORTB_DO
*   PORTD_DO
*   CDC_I2C_SDA_RD
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_7_15_23_31_AND_SEL for pad COMMB_3,
* a separate receiver enable for the IO2STMR_7_15_23_31 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMB_3_IO2STMR_7_15_23_31_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_COMMB_3_IO2STMR_7_15_23_31_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMMB_3_IO2STMR_7_15_23_31_AND_SEL_POS 24
#define IAXXX_IO_CTRL_COMMB_3_IO2STMR_7_15_23_31_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMB_3_IO2STMR_7_15_23_31_AND_SEL_DECL 24

/*** IO_CTRL_COMMC_0 (0x5002204c) ***/
/* This register is used for pad COMMC_0's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_COMMC_0_ADDR (0x5002204c)
#define IAXXX_IO_CTRL_COMMC_0_MASK_VAL 0x00c1c01f
#define IAXXX_IO_CTRL_COMMC_0_RMASK_VAL 0x00c1c01f
#define IAXXX_IO_CTRL_COMMC_0_WMASK_VAL 0x00c1c01f
#define IAXXX_IO_CTRL_COMMC_0_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad COMMC_0.
*   0x9: i2c3_clk
*   0xc: spi0_cs1_n
*   0xd: spi1_cs1_n
*   0x12: gpio_34
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_COMMC_0_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_COMMC_0_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_COMMC_0_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_COMMC_0_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_COMMC_0_MUX_SEL_DECL (4:0)

/*
* Function i2c3_clk can be selected through the following pads:
*   COMMC_0
*   COMMF_0
*   PORTA_CLK
*   CDC_INT_N
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C3_CLK_AND_SEL for pad COMMC_0, a separate
* receiver enable for the I2C3_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMC_0_I2C3_CLK_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_COMMC_0_I2C3_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMC_0_I2C3_CLK_AND_SEL_POS 14
#define IAXXX_IO_CTRL_COMMC_0_I2C3_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMC_0_I2C3_CLK_AND_SEL_DECL 14

/*
* Function uart0_cts_n can be selected through the following pads:
*   COMMA_0
*   COMMC_0
*   CDC_I2C_SCL
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control UART0_CTS_N_AND_SEL for pad COMMC_0, a
* separate receiver enable for the UART0_CTS_N pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMC_0_UART0_CTS_N_AND_SEL_MASK 0x00008000
#define IAXXX_IO_CTRL_COMMC_0_UART0_CTS_N_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMC_0_UART0_CTS_N_AND_SEL_POS 15
#define IAXXX_IO_CTRL_COMMC_0_UART0_CTS_N_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMC_0_UART0_CTS_N_AND_SEL_DECL 15

/*
* Function spi2_rxd can be selected through the following pads:
*   COMMC_0
*   PORTA_CLK
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SPI2_RXD_AND_SEL for pad COMMC_0, a separate
* receiver enable for the SPI2_RXD pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMC_0_SPI2_RXD_AND_SEL_MASK 0x00010000
#define IAXXX_IO_CTRL_COMMC_0_SPI2_RXD_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMC_0_SPI2_RXD_AND_SEL_POS 16
#define IAXXX_IO_CTRL_COMMC_0_SPI2_RXD_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMC_0_SPI2_RXD_AND_SEL_DECL 16

/*
* Function fi_24 can be selected through the following pads:
*   COMMC_0
*   PORTE_CLK
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control FI_24_AND_SEL for pad COMMC_0, a separate
* receiver enable for the FI_24 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMC_0_FI_24_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_COMMC_0_FI_24_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMC_0_FI_24_AND_SEL_POS 22
#define IAXXX_IO_CTRL_COMMC_0_FI_24_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMC_0_FI_24_AND_SEL_DECL 22

/*
* This field is used to control GPIO_34_AND_SEL for pad COMMC_0, a separate
* receiver enable for the GPIO_34 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMC_0_GPIO_34_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_COMMC_0_GPIO_34_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMMC_0_GPIO_34_AND_SEL_POS 23
#define IAXXX_IO_CTRL_COMMC_0_GPIO_34_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMC_0_GPIO_34_AND_SEL_DECL 23

/*** IO_CTRL_COMMC_1 (0x50022050) ***/
/* This register is used for pad COMMC_1's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_COMMC_1_ADDR (0x50022050)
#define IAXXX_IO_CTRL_COMMC_1_MASK_VAL 0x00c1401f
#define IAXXX_IO_CTRL_COMMC_1_RMASK_VAL 0x00c1401f
#define IAXXX_IO_CTRL_COMMC_1_WMASK_VAL 0x00c1401f
#define IAXXX_IO_CTRL_COMMC_1_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad COMMC_1.
*   0x9: i2c3_data
*   0xa: uart0_rts_n
*   0xb: spi2_clk
*   0xc: spi0_cs2_n
*   0xd: spi1_cs2_n
*   0x12: gpio_35
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_COMMC_1_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_COMMC_1_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_COMMC_1_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_COMMC_1_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_COMMC_1_MUX_SEL_DECL (4:0)

/*
* Function i2c3_data can be selected through the following pads:
*   COMMC_1
*   COMMF_1
*   PORTA_FS
*   CDC_RESET_N
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C3_DATA_AND_SEL for pad COMMC_1, a
* separate receiver enable for the I2C3_DATA pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMC_1_I2C3_DATA_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_COMMC_1_I2C3_DATA_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMC_1_I2C3_DATA_AND_SEL_POS 14
#define IAXXX_IO_CTRL_COMMC_1_I2C3_DATA_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMC_1_I2C3_DATA_AND_SEL_DECL 14

/*
* Function spi2_clk can be selected through the following pads:
*   COMMC_1
*   PORTA_FS
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SPI2_CLK_AND_SEL for pad COMMC_1, a separate
* receiver enable for the SPI2_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMC_1_SPI2_CLK_AND_SEL_MASK 0x00010000
#define IAXXX_IO_CTRL_COMMC_1_SPI2_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMC_1_SPI2_CLK_AND_SEL_POS 16
#define IAXXX_IO_CTRL_COMMC_1_SPI2_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMC_1_SPI2_CLK_AND_SEL_DECL 16

/*
* Function fi_25 can be selected through the following pads:
*   COMMC_1
*   PORTE_FS
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control FI_25_AND_SEL for pad COMMC_1, a separate
* receiver enable for the FI_25 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMC_1_FI_25_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_COMMC_1_FI_25_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMC_1_FI_25_AND_SEL_POS 22
#define IAXXX_IO_CTRL_COMMC_1_FI_25_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMC_1_FI_25_AND_SEL_DECL 22

/*
* This field is used to control GPIO_35_AND_SEL for pad COMMC_1, a separate
* receiver enable for the GPIO_35 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMC_1_GPIO_35_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_COMMC_1_GPIO_35_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMMC_1_GPIO_35_AND_SEL_POS 23
#define IAXXX_IO_CTRL_COMMC_1_GPIO_35_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMC_1_GPIO_35_AND_SEL_DECL 23

/*** IO_CTRL_COMMC_2 (0x50022054) ***/
/* This register is used for pad COMMC_2's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_COMMC_2_ADDR (0x50022054)
#define IAXXX_IO_CTRL_COMMC_2_MASK_VAL 0x00c1c01f
#define IAXXX_IO_CTRL_COMMC_2_RMASK_VAL 0x00c1c01f
#define IAXXX_IO_CTRL_COMMC_2_WMASK_VAL 0x00c1c01f
#define IAXXX_IO_CTRL_COMMC_2_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad COMMC_2.
*   0x9: i2c4_clk
*   0xb: spi2_cs0_n
*   0xc: spi0_cs1_n
*   0xd: spi1_cs1_n
*   0x12: gpio_36
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_COMMC_2_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_COMMC_2_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_COMMC_2_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_COMMC_2_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_COMMC_2_MUX_SEL_DECL (4:0)

/*
* Function i2c4_clk can be selected through the following pads:
*   COMMC_2
*   COMMF_2
*   PORTA_DI
*   CDC_PWRON
*   CDC_I2C_SCL
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C4_CLK_AND_SEL for pad COMMC_2, a separate
* receiver enable for the I2C4_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMC_2_I2C4_CLK_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_COMMC_2_I2C4_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMC_2_I2C4_CLK_AND_SEL_POS 14
#define IAXXX_IO_CTRL_COMMC_2_I2C4_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMC_2_I2C4_CLK_AND_SEL_DECL 14

/*
* Function uart0_sin can be selected through the following pads:
*   COMMA_2
*   COMMC_2
*   COMMD_0
*   PORTD_DI
*   PORTE_DI
*   CDC_PWRON
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control UART0_SIN_AND_SEL for pad COMMC_2, a
* separate receiver enable for the UART0_SIN pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMC_2_UART0_SIN_AND_SEL_MASK 0x00008000
#define IAXXX_IO_CTRL_COMMC_2_UART0_SIN_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMC_2_UART0_SIN_AND_SEL_POS 15
#define IAXXX_IO_CTRL_COMMC_2_UART0_SIN_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMC_2_UART0_SIN_AND_SEL_DECL 15

/*
* Function spi2_cs0_n can be selected through the following pads:
*   COMMC_2
*   PORTA_DI
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SPI2_CS0_N_AND_SEL for pad COMMC_2, a
* separate receiver enable for the SPI2_CS0_N pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMC_2_SPI2_CS0_N_AND_SEL_MASK 0x00010000
#define IAXXX_IO_CTRL_COMMC_2_SPI2_CS0_N_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMC_2_SPI2_CS0_N_AND_SEL_POS 16
#define IAXXX_IO_CTRL_COMMC_2_SPI2_CS0_N_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMC_2_SPI2_CS0_N_AND_SEL_DECL 16

/*
* Function fi_26 can be selected through the following pads:
*   COMMC_2
*   PORTE_DI
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control FI_26_AND_SEL for pad COMMC_2, a separate
* receiver enable for the FI_26 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMC_2_FI_26_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_COMMC_2_FI_26_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMC_2_FI_26_AND_SEL_POS 22
#define IAXXX_IO_CTRL_COMMC_2_FI_26_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMC_2_FI_26_AND_SEL_DECL 22

/*
* This field is used to control GPIO_36_AND_SEL for pad COMMC_2, a separate
* receiver enable for the GPIO_36 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMC_2_GPIO_36_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_COMMC_2_GPIO_36_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMMC_2_GPIO_36_AND_SEL_POS 23
#define IAXXX_IO_CTRL_COMMC_2_GPIO_36_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMC_2_GPIO_36_AND_SEL_DECL 23

/*** IO_CTRL_COMMC_3 (0x50022058) ***/
/* This register is used for pad COMMC_3's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_COMMC_3_ADDR (0x50022058)
#define IAXXX_IO_CTRL_COMMC_3_MASK_VAL 0x00c0401f
#define IAXXX_IO_CTRL_COMMC_3_RMASK_VAL 0x00c0401f
#define IAXXX_IO_CTRL_COMMC_3_WMASK_VAL 0x00c0401f
#define IAXXX_IO_CTRL_COMMC_3_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad COMMC_3.
*   0x9: i2c4_data
*   0xa: uart0_sout
*   0xb: spi2_txd
*   0xc: spi0_cs3_n
*   0xd: spi1_cs3_n
*   0x12: gpio_37
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_COMMC_3_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_COMMC_3_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_COMMC_3_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_COMMC_3_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_COMMC_3_MUX_SEL_DECL (4:0)

/*
* Function i2c4_data can be selected through the following pads:
*   COMMC_3
*   COMMF_3
*   PORTA_DO
*   CDC_I2C_SDA_RD
*   CDC_I2C_SDA_WR
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C4_DATA_AND_SEL for pad COMMC_3, a
* separate receiver enable for the I2C4_DATA pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMC_3_I2C4_DATA_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_COMMC_3_I2C4_DATA_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMC_3_I2C4_DATA_AND_SEL_POS 14
#define IAXXX_IO_CTRL_COMMC_3_I2C4_DATA_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMC_3_I2C4_DATA_AND_SEL_DECL 14

/*
* Function fi_27 can be selected through the following pads:
*   COMMC_3
*   PORTE_DO
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control FI_27_AND_SEL for pad COMMC_3, a separate
* receiver enable for the FI_27 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMC_3_FI_27_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_COMMC_3_FI_27_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMC_3_FI_27_AND_SEL_POS 22
#define IAXXX_IO_CTRL_COMMC_3_FI_27_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMC_3_FI_27_AND_SEL_DECL 22

/*
* This field is used to control GPIO_37_AND_SEL for pad COMMC_3, a separate
* receiver enable for the GPIO_37 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMC_3_GPIO_37_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_COMMC_3_GPIO_37_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMMC_3_GPIO_37_AND_SEL_POS 23
#define IAXXX_IO_CTRL_COMMC_3_GPIO_37_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMC_3_GPIO_37_AND_SEL_DECL 23

/*** IO_CTRL_COMMD_0 (0x5002205c) ***/
/* This register is used for pad COMMD_0's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_COMMD_0_ADDR (0x5002205c)
#define IAXXX_IO_CTRL_COMMD_0_MASK_VAL 0x00c1c01f
#define IAXXX_IO_CTRL_COMMD_0_RMASK_VAL 0x00c1c01f
#define IAXXX_IO_CTRL_COMMD_0_WMASK_VAL 0x00c1c01f
#define IAXXX_IO_CTRL_COMMD_0_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad COMMD_0.
*   0x9: i2c0_clk
*   0xc: spi0_cs1_n
*   0xd: spi1_cs1_n
*   0xe: spi2_cs1_n
*   0x12: gpio_38
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_COMMD_0_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_COMMD_0_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_COMMD_0_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_COMMD_0_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_COMMD_0_MUX_SEL_DECL (4:0)

/*
* Function i2c0_clk can be selected through the following pads:
*   COMMA_0
*   COMMD_0
*   PORTD_CLK
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C0_CLK_AND_SEL for pad COMMD_0, a separate
* receiver enable for the I2C0_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMD_0_I2C0_CLK_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_COMMD_0_I2C0_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMD_0_I2C0_CLK_AND_SEL_POS 14
#define IAXXX_IO_CTRL_COMMD_0_I2C0_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMD_0_I2C0_CLK_AND_SEL_DECL 14

/*
* Function uart0_sin can be selected through the following pads:
*   COMMA_2
*   COMMC_2
*   COMMD_0
*   PORTD_DI
*   PORTE_DI
*   CDC_PWRON
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control UART0_SIN_AND_SEL for pad COMMD_0, a
* separate receiver enable for the UART0_SIN pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMD_0_UART0_SIN_AND_SEL_MASK 0x00008000
#define IAXXX_IO_CTRL_COMMD_0_UART0_SIN_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMD_0_UART0_SIN_AND_SEL_POS 15
#define IAXXX_IO_CTRL_COMMD_0_UART0_SIN_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMD_0_UART0_SIN_AND_SEL_DECL 15

/*
* Function spi0_rxd can be selected through the following pads:
*   COMMA_0
*   COMMD_0
*   PORTD_CLK
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SPI0_RXD_AND_SEL for pad COMMD_0, a separate
* receiver enable for the SPI0_RXD pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMD_0_SPI0_RXD_AND_SEL_MASK 0x00010000
#define IAXXX_IO_CTRL_COMMD_0_SPI0_RXD_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMD_0_SPI0_RXD_AND_SEL_POS 16
#define IAXXX_IO_CTRL_COMMD_0_SPI0_RXD_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMD_0_SPI0_RXD_AND_SEL_DECL 16

/*
* This field is used to control FI_28_AND_SEL for pad COMMD_0, a separate
* receiver enable for the FI_28 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMD_0_FI_28_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_COMMD_0_FI_28_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMD_0_FI_28_AND_SEL_POS 22
#define IAXXX_IO_CTRL_COMMD_0_FI_28_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMD_0_FI_28_AND_SEL_DECL 22

/*
* This field is used to control GPIO_38_AND_SEL for pad COMMD_0, a separate
* receiver enable for the GPIO_38 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMD_0_GPIO_38_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_COMMD_0_GPIO_38_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMMD_0_GPIO_38_AND_SEL_POS 23
#define IAXXX_IO_CTRL_COMMD_0_GPIO_38_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMD_0_GPIO_38_AND_SEL_DECL 23

/*** IO_CTRL_COMMD_1 (0x50022060) ***/
/* This register is used for pad COMMD_1's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_COMMD_1_ADDR (0x50022060)
#define IAXXX_IO_CTRL_COMMD_1_MASK_VAL 0x00c1401f
#define IAXXX_IO_CTRL_COMMD_1_RMASK_VAL 0x00c1401f
#define IAXXX_IO_CTRL_COMMD_1_WMASK_VAL 0x00c1401f
#define IAXXX_IO_CTRL_COMMD_1_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad COMMD_1.
*   0x9: i2c0_data
*   0xa: uart0_sout
*   0xb: spi0_clk
*   0xc: spi0_cs2_n
*   0xd: spi1_cs2_n
*   0xe: spi2_cs2_n
*   0x12: gpio_39
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_COMMD_1_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_COMMD_1_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_COMMD_1_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_COMMD_1_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_COMMD_1_MUX_SEL_DECL (4:0)

/*
* Function i2c0_data can be selected through the following pads:
*   COMMA_1
*   COMMD_1
*   PORTD_FS
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C0_DATA_AND_SEL for pad COMMD_1, a
* separate receiver enable for the I2C0_DATA pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMD_1_I2C0_DATA_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_COMMD_1_I2C0_DATA_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMD_1_I2C0_DATA_AND_SEL_POS 14
#define IAXXX_IO_CTRL_COMMD_1_I2C0_DATA_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMD_1_I2C0_DATA_AND_SEL_DECL 14

/*
* Function spi0_clk can be selected through the following pads:
*   COMMA_1
*   COMMD_1
*   PORTD_FS
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SPI0_CLK_AND_SEL for pad COMMD_1, a separate
* receiver enable for the SPI0_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMD_1_SPI0_CLK_AND_SEL_MASK 0x00010000
#define IAXXX_IO_CTRL_COMMD_1_SPI0_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMD_1_SPI0_CLK_AND_SEL_POS 16
#define IAXXX_IO_CTRL_COMMD_1_SPI0_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMD_1_SPI0_CLK_AND_SEL_DECL 16

/*
* This field is used to control FI_29_AND_SEL for pad COMMD_1, a separate
* receiver enable for the FI_29 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMD_1_FI_29_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_COMMD_1_FI_29_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMD_1_FI_29_AND_SEL_POS 22
#define IAXXX_IO_CTRL_COMMD_1_FI_29_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMD_1_FI_29_AND_SEL_DECL 22

/*
* This field is used to control GPIO_39_AND_SEL for pad COMMD_1, a separate
* receiver enable for the GPIO_39 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMD_1_GPIO_39_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_COMMD_1_GPIO_39_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMMD_1_GPIO_39_AND_SEL_POS 23
#define IAXXX_IO_CTRL_COMMD_1_GPIO_39_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMD_1_GPIO_39_AND_SEL_DECL 23

/*** IO_CTRL_COMMD_2 (0x50022064) ***/
/* This register is used for pad COMMD_2's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_COMMD_2_ADDR (0x50022064)
#define IAXXX_IO_CTRL_COMMD_2_MASK_VAL 0x00c1901f
#define IAXXX_IO_CTRL_COMMD_2_RMASK_VAL 0x00c1901f
#define IAXXX_IO_CTRL_COMMD_2_WMASK_VAL 0x00c1901f
#define IAXXX_IO_CTRL_COMMD_2_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad COMMD_2.
*   0x7: swrm_clk
*   0xb: spi0_cs0_n
*   0xc: spi0_cs1_n
*   0xd: spi1_cs1_n
*   0xe: spi2_cs1_n
*   0x12: gpio_40
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_COMMD_2_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_COMMD_2_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_COMMD_2_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_COMMD_2_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_COMMD_2_MUX_SEL_DECL (4:0)

/*
* Function swrm_clk can be selected through the following pads:
*   COMMD_2
*   COMMF_2
*   PORTB_CLK
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SWRM_CLK_AND_SEL for pad COMMD_2, a separate
* receiver enable for the SWRM_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMD_2_SWRM_CLK_AND_SEL_MASK 0x00001000
#define IAXXX_IO_CTRL_COMMD_2_SWRM_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMD_2_SWRM_CLK_AND_SEL_POS 12
#define IAXXX_IO_CTRL_COMMD_2_SWRM_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMD_2_SWRM_CLK_AND_SEL_DECL 12

/*
* Function uart1_sin can be selected through the following pads:
*   COMMB_2
*   COMMD_2
*   PORTD_CLK
*   PORTE_CLK
*   CDC_INT_N
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control UART1_SIN_AND_SEL for pad COMMD_2, a
* separate receiver enable for the UART1_SIN pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMD_2_UART1_SIN_AND_SEL_MASK 0x00008000
#define IAXXX_IO_CTRL_COMMD_2_UART1_SIN_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMD_2_UART1_SIN_AND_SEL_POS 15
#define IAXXX_IO_CTRL_COMMD_2_UART1_SIN_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMD_2_UART1_SIN_AND_SEL_DECL 15

/*
* Function spi0_cs0_n can be selected through the following pads:
*   COMMA_2
*   COMMD_2
*   PORTD_DI
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SPI0_CS0_N_AND_SEL for pad COMMD_2, a
* separate receiver enable for the SPI0_CS0_N pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMD_2_SPI0_CS0_N_AND_SEL_MASK 0x00010000
#define IAXXX_IO_CTRL_COMMD_2_SPI0_CS0_N_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMD_2_SPI0_CS0_N_AND_SEL_POS 16
#define IAXXX_IO_CTRL_COMMD_2_SPI0_CS0_N_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMD_2_SPI0_CS0_N_AND_SEL_DECL 16

/*
* This field is used to control FI_30_AND_SEL for pad COMMD_2, a separate
* receiver enable for the FI_30 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMD_2_FI_30_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_COMMD_2_FI_30_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMD_2_FI_30_AND_SEL_POS 22
#define IAXXX_IO_CTRL_COMMD_2_FI_30_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMD_2_FI_30_AND_SEL_DECL 22

/*
* This field is used to control GPIO_40_AND_SEL for pad COMMD_2, a separate
* receiver enable for the GPIO_40 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMD_2_GPIO_40_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_COMMD_2_GPIO_40_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMMD_2_GPIO_40_AND_SEL_POS 23
#define IAXXX_IO_CTRL_COMMD_2_GPIO_40_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMD_2_GPIO_40_AND_SEL_DECL 23

/*** IO_CTRL_COMMD_3 (0x50022068) ***/
/* This register is used for pad COMMD_3's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_COMMD_3_ADDR (0x50022068)
#define IAXXX_IO_CTRL_COMMD_3_MASK_VAL 0x00c0101f
#define IAXXX_IO_CTRL_COMMD_3_RMASK_VAL 0x00c0101f
#define IAXXX_IO_CTRL_COMMD_3_WMASK_VAL 0x00c0101f
#define IAXXX_IO_CTRL_COMMD_3_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad COMMD_3.
*   0x7: swrm_data
*   0xa: uart1_sout
*   0xb: spi0_txd
*   0xc: spi0_cs3_n
*   0xd: spi1_cs3_n
*   0xe: spi2_cs3_n
*   0x12: gpio_41
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_COMMD_3_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_COMMD_3_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_COMMD_3_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_COMMD_3_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_COMMD_3_MUX_SEL_DECL (4:0)

/*
* Function swrm_data can be selected through the following pads:
*   COMMD_3
*   COMMF_3
*   PORTB_DI
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SWRM_DATA_AND_SEL for pad COMMD_3, a
* separate receiver enable for the SWRM_DATA pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMD_3_SWRM_DATA_AND_SEL_MASK 0x00001000
#define IAXXX_IO_CTRL_COMMD_3_SWRM_DATA_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMD_3_SWRM_DATA_AND_SEL_POS 12
#define IAXXX_IO_CTRL_COMMD_3_SWRM_DATA_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMD_3_SWRM_DATA_AND_SEL_DECL 12

/*
* This field is used to control FI_31_AND_SEL for pad COMMD_3, a separate
* receiver enable for the FI_31 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMD_3_FI_31_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_COMMD_3_FI_31_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMD_3_FI_31_AND_SEL_POS 22
#define IAXXX_IO_CTRL_COMMD_3_FI_31_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMD_3_FI_31_AND_SEL_DECL 22

/*
* This field is used to control GPIO_41_AND_SEL for pad COMMD_3, a separate
* receiver enable for the GPIO_41 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMD_3_GPIO_41_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_COMMD_3_GPIO_41_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMMD_3_GPIO_41_AND_SEL_POS 23
#define IAXXX_IO_CTRL_COMMD_3_GPIO_41_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMD_3_GPIO_41_AND_SEL_DECL 23

/*** IO_CTRL_COMME_0 (0x5002206c) ***/
/* This register is used for pad COMME_0's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_COMME_0_ADDR (0x5002206c)
#define IAXXX_IO_CTRL_COMME_0_MASK_VAL 0x0080401f
#define IAXXX_IO_CTRL_COMME_0_RMASK_VAL 0x0080401f
#define IAXXX_IO_CTRL_COMME_0_WMASK_VAL 0x0080401f
#define IAXXX_IO_CTRL_COMME_0_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad COMME_0.
*   0x9: i2c1_clk
*   0xc: spi0_cs1_n
*   0xd: spi1_cs1_n
*   0xe: spi2_cs1_n
*   0x12: gpio_42
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_COMME_0_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_COMME_0_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_COMME_0_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_COMME_0_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_COMME_0_MUX_SEL_DECL (4:0)

/*
* Function i2c1_clk can be selected through the following pads:
*   COMMB_0
*   COMME_0
*   PORTE_CLK
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C1_CLK_AND_SEL for pad COMME_0, a separate
* receiver enable for the I2C1_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMME_0_I2C1_CLK_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_COMME_0_I2C1_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMME_0_I2C1_CLK_AND_SEL_POS 14
#define IAXXX_IO_CTRL_COMME_0_I2C1_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMME_0_I2C1_CLK_AND_SEL_DECL 14

/*
* This field is used to control GPIO_42_AND_SEL for pad COMME_0, a separate
* receiver enable for the GPIO_42 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMME_0_GPIO_42_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_COMME_0_GPIO_42_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMME_0_GPIO_42_AND_SEL_POS 23
#define IAXXX_IO_CTRL_COMME_0_GPIO_42_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMME_0_GPIO_42_AND_SEL_DECL 23

/*** IO_CTRL_COMME_1 (0x50022070) ***/
/* This register is used for pad COMME_1's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_COMME_1_ADDR (0x50022070)
#define IAXXX_IO_CTRL_COMME_1_MASK_VAL 0x0080401f
#define IAXXX_IO_CTRL_COMME_1_RMASK_VAL 0x0080401f
#define IAXXX_IO_CTRL_COMME_1_WMASK_VAL 0x0080401f
#define IAXXX_IO_CTRL_COMME_1_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad COMME_1.
*   0x9: i2c1_data
*   0xc: spi0_cs2_n
*   0xd: spi1_cs2_n
*   0xe: spi2_cs2_n
*   0x12: gpio_43
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_COMME_1_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_COMME_1_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_COMME_1_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_COMME_1_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_COMME_1_MUX_SEL_DECL (4:0)

/*
* Function i2c1_data can be selected through the following pads:
*   COMMB_1
*   COMME_1
*   PORTE_FS
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C1_DATA_AND_SEL for pad COMME_1, a
* separate receiver enable for the I2C1_DATA pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMME_1_I2C1_DATA_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_COMME_1_I2C1_DATA_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMME_1_I2C1_DATA_AND_SEL_POS 14
#define IAXXX_IO_CTRL_COMME_1_I2C1_DATA_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMME_1_I2C1_DATA_AND_SEL_DECL 14

/*
* This field is used to control GPIO_43_AND_SEL for pad COMME_1, a separate
* receiver enable for the GPIO_43 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMME_1_GPIO_43_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_COMME_1_GPIO_43_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMME_1_GPIO_43_AND_SEL_POS 23
#define IAXXX_IO_CTRL_COMME_1_GPIO_43_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMME_1_GPIO_43_AND_SEL_DECL 23

/*** IO_CTRL_COMME_2 (0x50022074) ***/
/* This register is used for pad COMME_2's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_COMME_2_ADDR (0x50022074)
#define IAXXX_IO_CTRL_COMME_2_MASK_VAL 0x0080401f
#define IAXXX_IO_CTRL_COMME_2_RMASK_VAL 0x0080401f
#define IAXXX_IO_CTRL_COMME_2_WMASK_VAL 0x0080401f
#define IAXXX_IO_CTRL_COMME_2_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad COMME_2.
*   0x9: i2c2_clk
*   0xc: spi0_cs1_n
*   0xd: spi1_cs1_n
*   0xe: spi2_cs1_n
*   0x12: gpio_44
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_COMME_2_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_COMME_2_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_COMME_2_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_COMME_2_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_COMME_2_MUX_SEL_DECL (4:0)

/*
* Function i2c2_clk can be selected through the following pads:
*   COMMB_2
*   COMME_2
*   PORTE_DI
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C2_CLK_AND_SEL for pad COMME_2, a separate
* receiver enable for the I2C2_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMME_2_I2C2_CLK_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_COMME_2_I2C2_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMME_2_I2C2_CLK_AND_SEL_POS 14
#define IAXXX_IO_CTRL_COMME_2_I2C2_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMME_2_I2C2_CLK_AND_SEL_DECL 14

/*
* This field is used to control GPIO_44_AND_SEL for pad COMME_2, a separate
* receiver enable for the GPIO_44 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMME_2_GPIO_44_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_COMME_2_GPIO_44_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMME_2_GPIO_44_AND_SEL_POS 23
#define IAXXX_IO_CTRL_COMME_2_GPIO_44_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMME_2_GPIO_44_AND_SEL_DECL 23

/*** IO_CTRL_COMME_3 (0x50022078) ***/
/* This register is used for pad COMME_3's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_COMME_3_ADDR (0x50022078)
#define IAXXX_IO_CTRL_COMME_3_MASK_VAL 0x0080401f
#define IAXXX_IO_CTRL_COMME_3_RMASK_VAL 0x0080401f
#define IAXXX_IO_CTRL_COMME_3_WMASK_VAL 0x0080401f
#define IAXXX_IO_CTRL_COMME_3_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad COMME_3.
*   0x9: i2c2_data
*   0xc: spi0_cs3_n
*   0xd: spi1_cs3_n
*   0xe: spi2_cs3_n
*   0x12: gpio_45
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_COMME_3_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_COMME_3_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_COMME_3_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_COMME_3_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_COMME_3_MUX_SEL_DECL (4:0)

/*
* Function i2c2_data can be selected through the following pads:
*   COMMB_3
*   COMME_3
*   PORTE_DO
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C2_DATA_AND_SEL for pad COMME_3, a
* separate receiver enable for the I2C2_DATA pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMME_3_I2C2_DATA_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_COMME_3_I2C2_DATA_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMME_3_I2C2_DATA_AND_SEL_POS 14
#define IAXXX_IO_CTRL_COMME_3_I2C2_DATA_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMME_3_I2C2_DATA_AND_SEL_DECL 14

/*
* This field is used to control GPIO_45_AND_SEL for pad COMME_3, a separate
* receiver enable for the GPIO_45 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMME_3_GPIO_45_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_COMME_3_GPIO_45_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMME_3_GPIO_45_AND_SEL_POS 23
#define IAXXX_IO_CTRL_COMME_3_GPIO_45_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMME_3_GPIO_45_AND_SEL_DECL 23

/*** IO_CTRL_COMMF_0 (0x5002207c) ***/
/* This register is used for pad COMMF_0's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_COMMF_0_ADDR (0x5002207c)
#define IAXXX_IO_CTRL_COMMF_0_MASK_VAL 0x0080401f
#define IAXXX_IO_CTRL_COMMF_0_RMASK_VAL 0x0080401f
#define IAXXX_IO_CTRL_COMMF_0_WMASK_VAL 0x0080401f
#define IAXXX_IO_CTRL_COMMF_0_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad COMMF_0.
*   0x9: i2c3_clk
*   0xc: spi0_cs1_n
*   0xd: spi1_cs1_n
*   0xe: spi2_cs1_n
*   0x12: gpio_46
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_COMMF_0_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_COMMF_0_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_COMMF_0_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_COMMF_0_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_COMMF_0_MUX_SEL_DECL (4:0)

/*
* Function i2c3_clk can be selected through the following pads:
*   COMMC_0
*   COMMF_0
*   PORTA_CLK
*   CDC_INT_N
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C3_CLK_AND_SEL for pad COMMF_0, a separate
* receiver enable for the I2C3_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMF_0_I2C3_CLK_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_COMMF_0_I2C3_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMF_0_I2C3_CLK_AND_SEL_POS 14
#define IAXXX_IO_CTRL_COMMF_0_I2C3_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMF_0_I2C3_CLK_AND_SEL_DECL 14

/*
* This field is used to control GPIO_46_AND_SEL for pad COMMF_0, a separate
* receiver enable for the GPIO_46 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMF_0_GPIO_46_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_COMMF_0_GPIO_46_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMMF_0_GPIO_46_AND_SEL_POS 23
#define IAXXX_IO_CTRL_COMMF_0_GPIO_46_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMF_0_GPIO_46_AND_SEL_DECL 23

/*** IO_CTRL_COMMF_1 (0x50022080) ***/
/* This register is used for pad COMMF_1's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_COMMF_1_ADDR (0x50022080)
#define IAXXX_IO_CTRL_COMMF_1_MASK_VAL 0x0080401f
#define IAXXX_IO_CTRL_COMMF_1_RMASK_VAL 0x0080401f
#define IAXXX_IO_CTRL_COMMF_1_WMASK_VAL 0x0080401f
#define IAXXX_IO_CTRL_COMMF_1_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad COMMF_1.
*   0x9: i2c3_data
*   0xc: spi0_cs2_n
*   0xd: spi1_cs2_n
*   0xe: spi2_cs2_n
*   0x12: gpio_47
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_COMMF_1_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_COMMF_1_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_COMMF_1_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_COMMF_1_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_COMMF_1_MUX_SEL_DECL (4:0)

/*
* Function i2c3_data can be selected through the following pads:
*   COMMC_1
*   COMMF_1
*   PORTA_FS
*   CDC_RESET_N
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C3_DATA_AND_SEL for pad COMMF_1, a
* separate receiver enable for the I2C3_DATA pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMF_1_I2C3_DATA_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_COMMF_1_I2C3_DATA_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMF_1_I2C3_DATA_AND_SEL_POS 14
#define IAXXX_IO_CTRL_COMMF_1_I2C3_DATA_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMF_1_I2C3_DATA_AND_SEL_DECL 14

/*
* This field is used to control GPIO_47_AND_SEL for pad COMMF_1, a separate
* receiver enable for the GPIO_47 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMF_1_GPIO_47_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_COMMF_1_GPIO_47_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMMF_1_GPIO_47_AND_SEL_POS 23
#define IAXXX_IO_CTRL_COMMF_1_GPIO_47_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMF_1_GPIO_47_AND_SEL_DECL 23

/*** IO_CTRL_COMMF_2 (0x50022084) ***/
/* This register is used for pad COMMF_2's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_COMMF_2_ADDR (0x50022084)
#define IAXXX_IO_CTRL_COMMF_2_MASK_VAL 0x0080501f
#define IAXXX_IO_CTRL_COMMF_2_RMASK_VAL 0x0080501f
#define IAXXX_IO_CTRL_COMMF_2_WMASK_VAL 0x0080501f
#define IAXXX_IO_CTRL_COMMF_2_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad COMMF_2.
*   0x7: swrm_clk
*   0x9: i2c4_clk
*   0xc: spi0_cs1_n
*   0xd: spi1_cs1_n
*   0xe: spi2_cs1_n
*   0x12: gpio_48
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_COMMF_2_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_COMMF_2_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_COMMF_2_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_COMMF_2_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_COMMF_2_MUX_SEL_DECL (4:0)

/*
* Function swrm_clk can be selected through the following pads:
*   COMMD_2
*   COMMF_2
*   PORTB_CLK
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SWRM_CLK_AND_SEL for pad COMMF_2, a separate
* receiver enable for the SWRM_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMF_2_SWRM_CLK_AND_SEL_MASK 0x00001000
#define IAXXX_IO_CTRL_COMMF_2_SWRM_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMF_2_SWRM_CLK_AND_SEL_POS 12
#define IAXXX_IO_CTRL_COMMF_2_SWRM_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMF_2_SWRM_CLK_AND_SEL_DECL 12

/*
* Function i2c4_clk can be selected through the following pads:
*   COMMC_2
*   COMMF_2
*   PORTA_DI
*   CDC_PWRON
*   CDC_I2C_SCL
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C4_CLK_AND_SEL for pad COMMF_2, a separate
* receiver enable for the I2C4_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMF_2_I2C4_CLK_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_COMMF_2_I2C4_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMF_2_I2C4_CLK_AND_SEL_POS 14
#define IAXXX_IO_CTRL_COMMF_2_I2C4_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMF_2_I2C4_CLK_AND_SEL_DECL 14

/*
* This field is used to control GPIO_48_AND_SEL for pad COMMF_2, a separate
* receiver enable for the GPIO_48 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMF_2_GPIO_48_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_COMMF_2_GPIO_48_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMMF_2_GPIO_48_AND_SEL_POS 23
#define IAXXX_IO_CTRL_COMMF_2_GPIO_48_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMF_2_GPIO_48_AND_SEL_DECL 23

/*** IO_CTRL_COMMF_3 (0x50022088) ***/
/* This register is used for pad COMMF_3's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_COMMF_3_ADDR (0x50022088)
#define IAXXX_IO_CTRL_COMMF_3_MASK_VAL 0x0080501f
#define IAXXX_IO_CTRL_COMMF_3_RMASK_VAL 0x0080501f
#define IAXXX_IO_CTRL_COMMF_3_WMASK_VAL 0x0080501f
#define IAXXX_IO_CTRL_COMMF_3_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad COMMF_3.
*   0x7: swrm_data
*   0x9: i2c4_data
*   0xc: spi0_cs3_n
*   0xd: spi1_cs3_n
*   0xe: spi2_cs3_n
*   0x12: gpio_49
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_COMMF_3_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_COMMF_3_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_COMMF_3_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_COMMF_3_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_COMMF_3_MUX_SEL_DECL (4:0)

/*
* Function swrm_data can be selected through the following pads:
*   COMMD_3
*   COMMF_3
*   PORTB_DI
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SWRM_DATA_AND_SEL for pad COMMF_3, a
* separate receiver enable for the SWRM_DATA pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMF_3_SWRM_DATA_AND_SEL_MASK 0x00001000
#define IAXXX_IO_CTRL_COMMF_3_SWRM_DATA_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMF_3_SWRM_DATA_AND_SEL_POS 12
#define IAXXX_IO_CTRL_COMMF_3_SWRM_DATA_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMF_3_SWRM_DATA_AND_SEL_DECL 12

/*
* Function i2c4_data can be selected through the following pads:
*   COMMC_3
*   COMMF_3
*   PORTA_DO
*   CDC_I2C_SDA_RD
*   CDC_I2C_SDA_WR
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C4_DATA_AND_SEL for pad COMMF_3, a
* separate receiver enable for the I2C4_DATA pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMF_3_I2C4_DATA_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_COMMF_3_I2C4_DATA_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_COMMF_3_I2C4_DATA_AND_SEL_POS 14
#define IAXXX_IO_CTRL_COMMF_3_I2C4_DATA_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMF_3_I2C4_DATA_AND_SEL_DECL 14

/*
* This field is used to control GPIO_49_AND_SEL for pad COMMF_3, a separate
* receiver enable for the GPIO_49 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_COMMF_3_GPIO_49_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_COMMF_3_GPIO_49_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_COMMF_3_GPIO_49_AND_SEL_POS 23
#define IAXXX_IO_CTRL_COMMF_3_GPIO_49_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_COMMF_3_GPIO_49_AND_SEL_DECL 23

/*** IO_CTRL_INTR_API (0x5002208c) ***/
/* This register is used for pad INTR_API's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_INTR_API_ADDR (0x5002208c)
#define IAXXX_IO_CTRL_INTR_API_MASK_VAL 0x0180001f
#define IAXXX_IO_CTRL_INTR_API_RMASK_VAL 0x0180001f
#define IAXXX_IO_CTRL_INTR_API_WMASK_VAL 0x0180001f
#define IAXXX_IO_CTRL_INTR_API_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad INTR_API.
*   0x12: gpio_9
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_INTR_API_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_INTR_API_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_INTR_API_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_INTR_API_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_INTR_API_MUX_SEL_DECL (4:0)

/*
* This field is used to control GPIO_9_AND_SEL for pad INTR_API, a separate
* receiver enable for the GPIO_9 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_INTR_API_GPIO_9_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_INTR_API_GPIO_9_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_INTR_API_GPIO_9_AND_SEL_POS 23
#define IAXXX_IO_CTRL_INTR_API_GPIO_9_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_INTR_API_GPIO_9_AND_SEL_DECL 23

/*
* Function io2stmr_1_9_17_25 can be selected through the following pads:
*   COMMA_3
*   INTR_API
*   PORTA_FS
*   PORTC_FS
*   PORTE_FS
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_1_9_17_25_AND_SEL for pad INTR_API,
* a separate receiver enable for the IO2STMR_1_9_17_25 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_INTR_API_IO2STMR_1_9_17_25_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_INTR_API_IO2STMR_1_9_17_25_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_INTR_API_IO2STMR_1_9_17_25_AND_SEL_POS 24
#define IAXXX_IO_CTRL_INTR_API_IO2STMR_1_9_17_25_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_INTR_API_IO2STMR_1_9_17_25_AND_SEL_DECL 24

/*** IO_CTRL_INTR_EVNT (0x50022090) ***/
/* This register is used for pad INTR_EVNT's IO function control.Refer */
/* <a href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_INTR_EVNT_ADDR (0x50022090)
#define IAXXX_IO_CTRL_INTR_EVNT_MASK_VAL 0x0180001f
#define IAXXX_IO_CTRL_INTR_EVNT_RMASK_VAL 0x0180001f
#define IAXXX_IO_CTRL_INTR_EVNT_WMASK_VAL 0x0180001f
#define IAXXX_IO_CTRL_INTR_EVNT_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad INTR_EVNT.
*   0x8: cdc_int_fwd
*   0x12: gpio_2
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_INTR_EVNT_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_INTR_EVNT_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_INTR_EVNT_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_INTR_EVNT_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_INTR_EVNT_MUX_SEL_DECL (4:0)

/*
* This field is used to control GPIO_2_AND_SEL for pad INTR_EVNT, a separate
* receiver enable for the GPIO_2 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_INTR_EVNT_GPIO_2_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_INTR_EVNT_GPIO_2_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_INTR_EVNT_GPIO_2_AND_SEL_POS 23
#define IAXXX_IO_CTRL_INTR_EVNT_GPIO_2_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_INTR_EVNT_GPIO_2_AND_SEL_DECL 23

/*
* Function io2stmr_2_10_18_26 can be selected through the following pads:
*   COMMA_0
*   INTR_EVNT
*   PORTA_DI
*   PORTC_DI
*   PORTE_DI
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_2_10_18_26_AND_SEL for pad
* INTR_EVNT, a separate receiver enable for the IO2STMR_2_10_18_26 pad
* function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_INTR_EVNT_IO2STMR_2_10_18_26_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_INTR_EVNT_IO2STMR_2_10_18_26_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_INTR_EVNT_IO2STMR_2_10_18_26_AND_SEL_POS 24
#define IAXXX_IO_CTRL_INTR_EVNT_IO2STMR_2_10_18_26_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_INTR_EVNT_IO2STMR_2_10_18_26_AND_SEL_DECL 24

/*** IO_CTRL_JTAG_DBG_TCK (0x50022094) ***/
/* This register is used for pad JTAG_DBG_TCK's IO function control. Refer to */
/* the <a href="#io_pad_control_diagram">IO Function and PAD Control</a> */
/* to understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_JTAG_DBG_TCK_ADDR (0x50022094)
#define IAXXX_IO_CTRL_JTAG_DBG_TCK_MASK_VAL 0x00000000
#define IAXXX_IO_CTRL_JTAG_DBG_TCK_RMASK_VAL 0x00000000
#define IAXXX_IO_CTRL_JTAG_DBG_TCK_WMASK_VAL 0x00000000
#define IAXXX_IO_CTRL_JTAG_DBG_TCK_RESET_VAL 0x00000000

/*** IO_CTRL_JTAG_DBG_TDI (0x50022098) ***/
/* This register is used for pad JTAG_DBG_TDI's IO function control. Refer to */
/* the <a href="#io_pad_control_diagram">IO Function and PAD Control</a> */
/* to understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_JTAG_DBG_TDI_ADDR (0x50022098)
#define IAXXX_IO_CTRL_JTAG_DBG_TDI_MASK_VAL 0x00000000
#define IAXXX_IO_CTRL_JTAG_DBG_TDI_RMASK_VAL 0x00000000
#define IAXXX_IO_CTRL_JTAG_DBG_TDI_WMASK_VAL 0x00000000
#define IAXXX_IO_CTRL_JTAG_DBG_TDI_RESET_VAL 0x00000000

/*** IO_CTRL_JTAG_DBG_TDO (0x5002209c) ***/
/* This register is used for pad JTAG_DBG_TDO's IO function control. Refer to */
/* the <a href="#io_pad_control_diagram">IO Function and PAD Control</a> */
/* to understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_JTAG_DBG_TDO_ADDR (0x5002209c)
#define IAXXX_IO_CTRL_JTAG_DBG_TDO_MASK_VAL 0x00000000
#define IAXXX_IO_CTRL_JTAG_DBG_TDO_RMASK_VAL 0x00000000
#define IAXXX_IO_CTRL_JTAG_DBG_TDO_WMASK_VAL 0x00000000
#define IAXXX_IO_CTRL_JTAG_DBG_TDO_RESET_VAL 0x00000000

/*** IO_CTRL_JTAG_DBG_TMS (0x500220a0) ***/
/* This register is used for pad JTAG_DBG_TMS's IO function control. Refer to */
/* the <a href="#io_pad_control_diagram">IO Function and PAD Control</a> */
/* to understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_JTAG_DBG_TMS_ADDR (0x500220a0)
#define IAXXX_IO_CTRL_JTAG_DBG_TMS_MASK_VAL 0x00000000
#define IAXXX_IO_CTRL_JTAG_DBG_TMS_RMASK_VAL 0x00000000
#define IAXXX_IO_CTRL_JTAG_DBG_TMS_WMASK_VAL 0x00000000
#define IAXXX_IO_CTRL_JTAG_DBG_TMS_RESET_VAL 0x00000000

/*** IO_CTRL_PORTA_CLK (0x500220a4) ***/
/* This register is used for pad PORTA_CLK's IO function control.Refer */
/* <a href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_PORTA_CLK_ADDR (0x500220a4)
#define IAXXX_IO_CTRL_PORTA_CLK_MASK_VAL 0x01d1411f
#define IAXXX_IO_CTRL_PORTA_CLK_RMASK_VAL 0x01d1411f
#define IAXXX_IO_CTRL_PORTA_CLK_WMASK_VAL 0x01d1411f
#define IAXXX_IO_CTRL_PORTA_CLK_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad PORTA_CLK.
*   0x5: i2s_pcm0_bclk
*   0x6: i2s_cdc_clk
*   0x9: i2c3_clk
*   0xc: spi0_cs1_n
*   0xd: spi1_cs1_n
*   0xe: spi2_cs1_n
*   0x10: fo_8
*   0x12: gpio_16
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_PORTA_CLK_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_PORTA_CLK_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_PORTA_CLK_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_PORTA_CLK_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_PORTA_CLK_MUX_SEL_DECL (4:0)

/*
* This field is used to control PCM0_BCLK_AND_SEL for pad PORTA_CLK, a
* separate receiver enable for the PCM0_BCLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTA_CLK_PCM0_BCLK_AND_SEL_MASK 0x00000100
#define IAXXX_IO_CTRL_PORTA_CLK_PCM0_BCLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTA_CLK_PCM0_BCLK_AND_SEL_POS 8
#define IAXXX_IO_CTRL_PORTA_CLK_PCM0_BCLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTA_CLK_PCM0_BCLK_AND_SEL_DECL 8

/*
* Function i2c3_clk can be selected through the following pads:
*   COMMC_0
*   COMMF_0
*   PORTA_CLK
*   CDC_INT_N
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C3_CLK_AND_SEL for pad PORTA_CLK, a
* separate receiver enable for the I2C3_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTA_CLK_I2C3_CLK_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_PORTA_CLK_I2C3_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTA_CLK_I2C3_CLK_AND_SEL_POS 14
#define IAXXX_IO_CTRL_PORTA_CLK_I2C3_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTA_CLK_I2C3_CLK_AND_SEL_DECL 14

/*
* Function spi2_rxd can be selected through the following pads:
*   COMMC_0
*   PORTA_CLK
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SPI2_RXD_AND_SEL for pad PORTA_CLK, a
* separate receiver enable for the SPI2_RXD pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTA_CLK_SPI2_RXD_AND_SEL_MASK 0x00010000
#define IAXXX_IO_CTRL_PORTA_CLK_SPI2_RXD_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTA_CLK_SPI2_RXD_AND_SEL_POS 16
#define IAXXX_IO_CTRL_PORTA_CLK_SPI2_RXD_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTA_CLK_SPI2_RXD_AND_SEL_DECL 16

/*
* This field is used to control PORTA_CLK2CNR_AND_SEL for pad PORTA_CLK, a
* separate receiver enable for the PORTA_CLK2CNR pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTA_CLK_PORTA_CLK2CNR_AND_SEL_MASK 0x00100000
#define IAXXX_IO_CTRL_PORTA_CLK_PORTA_CLK2CNR_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTA_CLK_PORTA_CLK2CNR_AND_SEL_POS 20
#define IAXXX_IO_CTRL_PORTA_CLK_PORTA_CLK2CNR_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTA_CLK_PORTA_CLK2CNR_AND_SEL_DECL 20

/*
* Function fi_8 can be selected through the following pads:
*   PORTA_CLK
*   CDC_INT_N
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control FI_8_AND_SEL for pad PORTA_CLK, a separate
* receiver enable for the FI_8 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTA_CLK_FI_8_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_PORTA_CLK_FI_8_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTA_CLK_FI_8_AND_SEL_POS 22
#define IAXXX_IO_CTRL_PORTA_CLK_FI_8_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTA_CLK_FI_8_AND_SEL_DECL 22

/*
* This field is used to control GPIO_16_AND_SEL for pad PORTA_CLK, a
* separate receiver enable for the GPIO_16 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTA_CLK_GPIO_16_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_PORTA_CLK_GPIO_16_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_PORTA_CLK_GPIO_16_AND_SEL_POS 23
#define IAXXX_IO_CTRL_PORTA_CLK_GPIO_16_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTA_CLK_GPIO_16_AND_SEL_DECL 23

/*
* Function io2stmr_0_8_16_24 can be selected through the following pads:
*   COMMA_2
*   PORTA_CLK
*   PORTC_CLK
*   PORTE_CLK
*   CDC_PWRON
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_0_8_16_24_AND_SEL for pad PORTA_CLK,
* a separate receiver enable for the IO2STMR_0_8_16_24 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTA_CLK_IO2STMR_0_8_16_24_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_PORTA_CLK_IO2STMR_0_8_16_24_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTA_CLK_IO2STMR_0_8_16_24_AND_SEL_POS 24
#define IAXXX_IO_CTRL_PORTA_CLK_IO2STMR_0_8_16_24_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTA_CLK_IO2STMR_0_8_16_24_AND_SEL_DECL 24

/*** IO_CTRL_PORTA_DI (0x500220a8) ***/
/* This register is used for pad PORTA_DI's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_PORTA_DI_ADDR (0x500220a8)
#define IAXXX_IO_CTRL_PORTA_DI_MASK_VAL 0x01c1411f
#define IAXXX_IO_CTRL_PORTA_DI_RMASK_VAL 0x01c1411f
#define IAXXX_IO_CTRL_PORTA_DI_WMASK_VAL 0x01c1411f
#define IAXXX_IO_CTRL_PORTA_DI_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad PORTA_DI.
*   0x9: i2c4_clk
*   0xb: spi2_cs0_n
*   0xc: spi0_cs1_n
*   0xd: spi1_cs1_n
*   0xe: spi2_cs1_n
*   0x10: fo_10
*   0x12: gpio_18
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_PORTA_DI_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_PORTA_DI_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_PORTA_DI_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_PORTA_DI_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_PORTA_DI_MUX_SEL_DECL (4:0)

/*
* This field is used to control PCM0_DR_AND_SEL for pad PORTA_DI, a separate
* receiver enable for the PCM0_DR pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTA_DI_PCM0_DR_AND_SEL_MASK 0x00000100
#define IAXXX_IO_CTRL_PORTA_DI_PCM0_DR_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTA_DI_PCM0_DR_AND_SEL_POS 8
#define IAXXX_IO_CTRL_PORTA_DI_PCM0_DR_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTA_DI_PCM0_DR_AND_SEL_DECL 8

/*
* Function i2c4_clk can be selected through the following pads:
*   COMMC_2
*   COMMF_2
*   PORTA_DI
*   CDC_PWRON
*   CDC_I2C_SCL
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C4_CLK_AND_SEL for pad PORTA_DI, a
* separate receiver enable for the I2C4_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTA_DI_I2C4_CLK_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_PORTA_DI_I2C4_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTA_DI_I2C4_CLK_AND_SEL_POS 14
#define IAXXX_IO_CTRL_PORTA_DI_I2C4_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTA_DI_I2C4_CLK_AND_SEL_DECL 14

/*
* Function spi2_cs0_n can be selected through the following pads:
*   COMMC_2
*   PORTA_DI
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SPI2_CS0_N_AND_SEL for pad PORTA_DI, a
* separate receiver enable for the SPI2_CS0_N pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTA_DI_SPI2_CS0_N_AND_SEL_MASK 0x00010000
#define IAXXX_IO_CTRL_PORTA_DI_SPI2_CS0_N_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTA_DI_SPI2_CS0_N_AND_SEL_POS 16
#define IAXXX_IO_CTRL_PORTA_DI_SPI2_CS0_N_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTA_DI_SPI2_CS0_N_AND_SEL_DECL 16

/*
* This field is used to control FI_10_AND_SEL for pad PORTA_DI, a separate
* receiver enable for the FI_10 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTA_DI_FI_10_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_PORTA_DI_FI_10_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTA_DI_FI_10_AND_SEL_POS 22
#define IAXXX_IO_CTRL_PORTA_DI_FI_10_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTA_DI_FI_10_AND_SEL_DECL 22

/*
* This field is used to control GPIO_18_AND_SEL for pad PORTA_DI, a separate
* receiver enable for the GPIO_18 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTA_DI_GPIO_18_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_PORTA_DI_GPIO_18_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_PORTA_DI_GPIO_18_AND_SEL_POS 23
#define IAXXX_IO_CTRL_PORTA_DI_GPIO_18_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTA_DI_GPIO_18_AND_SEL_DECL 23

/*
* Function io2stmr_2_10_18_26 can be selected through the following pads:
*   COMMA_0
*   INTR_EVNT
*   PORTA_DI
*   PORTC_DI
*   PORTE_DI
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_2_10_18_26_AND_SEL for pad PORTA_DI,
* a separate receiver enable for the IO2STMR_2_10_18_26 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTA_DI_IO2STMR_2_10_18_26_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_PORTA_DI_IO2STMR_2_10_18_26_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTA_DI_IO2STMR_2_10_18_26_AND_SEL_POS 24
#define IAXXX_IO_CTRL_PORTA_DI_IO2STMR_2_10_18_26_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTA_DI_IO2STMR_2_10_18_26_AND_SEL_DECL 24

/*** IO_CTRL_PORTA_DO (0x500220ac) ***/
/* This register is used for pad PORTA_DO's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_PORTA_DO_ADDR (0x500220ac)
#define IAXXX_IO_CTRL_PORTA_DO_MASK_VAL 0x01c0401f
#define IAXXX_IO_CTRL_PORTA_DO_RMASK_VAL 0x01c0401f
#define IAXXX_IO_CTRL_PORTA_DO_WMASK_VAL 0x01c0401f
#define IAXXX_IO_CTRL_PORTA_DO_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad PORTA_DO.
*   0x3: pcm0_dt
*   0x9: i2c4_data
*   0xb: spi2_txd
*   0xc: spi0_cs3_n
*   0xd: spi1_cs3_n
*   0xe: spi2_cs3_n
*   0x10: fo_11
*   0x12: gpio_19
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_PORTA_DO_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_PORTA_DO_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_PORTA_DO_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_PORTA_DO_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_PORTA_DO_MUX_SEL_DECL (4:0)

/*
* Function i2c4_data can be selected through the following pads:
*   COMMC_3
*   COMMF_3
*   PORTA_DO
*   CDC_I2C_SDA_RD
*   CDC_I2C_SDA_WR
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C4_DATA_AND_SEL for pad PORTA_DO, a
* separate receiver enable for the I2C4_DATA pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTA_DO_I2C4_DATA_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_PORTA_DO_I2C4_DATA_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTA_DO_I2C4_DATA_AND_SEL_POS 14
#define IAXXX_IO_CTRL_PORTA_DO_I2C4_DATA_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTA_DO_I2C4_DATA_AND_SEL_DECL 14

/*
* This field is used to control FI_11_AND_SEL for pad PORTA_DO, a separate
* receiver enable for the FI_11 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTA_DO_FI_11_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_PORTA_DO_FI_11_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTA_DO_FI_11_AND_SEL_POS 22
#define IAXXX_IO_CTRL_PORTA_DO_FI_11_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTA_DO_FI_11_AND_SEL_DECL 22

/*
* This field is used to control GPIO_19_AND_SEL for pad PORTA_DO, a separate
* receiver enable for the GPIO_19 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTA_DO_GPIO_19_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_PORTA_DO_GPIO_19_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_PORTA_DO_GPIO_19_AND_SEL_POS 23
#define IAXXX_IO_CTRL_PORTA_DO_GPIO_19_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTA_DO_GPIO_19_AND_SEL_DECL 23

/*
* Function io2stmr_3_11_19_27 can be selected through the following pads:
*   COMMA_1
*   PORTA_DO
*   PORTC_DO
*   PORTE_DO
*   CDC_RESET_N
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_3_11_19_27_AND_SEL for pad PORTA_DO,
* a separate receiver enable for the IO2STMR_3_11_19_27 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTA_DO_IO2STMR_3_11_19_27_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_PORTA_DO_IO2STMR_3_11_19_27_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTA_DO_IO2STMR_3_11_19_27_AND_SEL_POS 24
#define IAXXX_IO_CTRL_PORTA_DO_IO2STMR_3_11_19_27_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTA_DO_IO2STMR_3_11_19_27_AND_SEL_DECL 24

/*** IO_CTRL_PORTA_FS (0x500220b0) ***/
/* This register is used for pad PORTA_FS's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_PORTA_FS_ADDR (0x500220b0)
#define IAXXX_IO_CTRL_PORTA_FS_MASK_VAL 0x01c1411f
#define IAXXX_IO_CTRL_PORTA_FS_RMASK_VAL 0x01c1411f
#define IAXXX_IO_CTRL_PORTA_FS_WMASK_VAL 0x01c1411f
#define IAXXX_IO_CTRL_PORTA_FS_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad PORTA_FS.
*   0x5: i2s_pcm0_fs
*   0x9: i2c3_data
*   0xb: spi2_clk
*   0xc: spi0_cs2_n
*   0xd: spi1_cs2_n
*   0xe: spi2_cs2_n
*   0x10: fo_9
*   0x12: gpio_17
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_PORTA_FS_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_PORTA_FS_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_PORTA_FS_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_PORTA_FS_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_PORTA_FS_MUX_SEL_DECL (4:0)

/*
* This field is used to control PCM0_FS_AND_SEL for pad PORTA_FS, a separate
* receiver enable for the PCM0_FS pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTA_FS_PCM0_FS_AND_SEL_MASK 0x00000100
#define IAXXX_IO_CTRL_PORTA_FS_PCM0_FS_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTA_FS_PCM0_FS_AND_SEL_POS 8
#define IAXXX_IO_CTRL_PORTA_FS_PCM0_FS_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTA_FS_PCM0_FS_AND_SEL_DECL 8

/*
* Function i2c3_data can be selected through the following pads:
*   COMMC_1
*   COMMF_1
*   PORTA_FS
*   CDC_RESET_N
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C3_DATA_AND_SEL for pad PORTA_FS, a
* separate receiver enable for the I2C3_DATA pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTA_FS_I2C3_DATA_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_PORTA_FS_I2C3_DATA_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTA_FS_I2C3_DATA_AND_SEL_POS 14
#define IAXXX_IO_CTRL_PORTA_FS_I2C3_DATA_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTA_FS_I2C3_DATA_AND_SEL_DECL 14

/*
* Function spi2_clk can be selected through the following pads:
*   COMMC_1
*   PORTA_FS
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SPI2_CLK_AND_SEL for pad PORTA_FS, a
* separate receiver enable for the SPI2_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTA_FS_SPI2_CLK_AND_SEL_MASK 0x00010000
#define IAXXX_IO_CTRL_PORTA_FS_SPI2_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTA_FS_SPI2_CLK_AND_SEL_POS 16
#define IAXXX_IO_CTRL_PORTA_FS_SPI2_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTA_FS_SPI2_CLK_AND_SEL_DECL 16

/*
* Function fi_9 can be selected through the following pads:
*   PORTA_FS
*   CDC_RESET_N
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control FI_9_AND_SEL for pad PORTA_FS, a separate
* receiver enable for the FI_9 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTA_FS_FI_9_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_PORTA_FS_FI_9_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTA_FS_FI_9_AND_SEL_POS 22
#define IAXXX_IO_CTRL_PORTA_FS_FI_9_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTA_FS_FI_9_AND_SEL_DECL 22

/*
* This field is used to control GPIO_17_AND_SEL for pad PORTA_FS, a separate
* receiver enable for the GPIO_17 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTA_FS_GPIO_17_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_PORTA_FS_GPIO_17_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_PORTA_FS_GPIO_17_AND_SEL_POS 23
#define IAXXX_IO_CTRL_PORTA_FS_GPIO_17_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTA_FS_GPIO_17_AND_SEL_DECL 23

/*
* Function io2stmr_1_9_17_25 can be selected through the following pads:
*   COMMA_3
*   INTR_API
*   PORTA_FS
*   PORTC_FS
*   PORTE_FS
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_1_9_17_25_AND_SEL for pad PORTA_FS,
* a separate receiver enable for the IO2STMR_1_9_17_25 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTA_FS_IO2STMR_1_9_17_25_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_PORTA_FS_IO2STMR_1_9_17_25_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTA_FS_IO2STMR_1_9_17_25_AND_SEL_POS 24
#define IAXXX_IO_CTRL_PORTA_FS_IO2STMR_1_9_17_25_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTA_FS_IO2STMR_1_9_17_25_AND_SEL_DECL 24

/*** IO_CTRL_PORTB_CLK (0x500220b4) ***/
/* This register is used for pad PORTB_CLK's IO function control.Refer */
/* <a href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_PORTB_CLK_ADDR (0x500220b4)
#define IAXXX_IO_CTRL_PORTB_CLK_MASK_VAL 0x01d0131f
#define IAXXX_IO_CTRL_PORTB_CLK_RMASK_VAL 0x01d0131f
#define IAXXX_IO_CTRL_PORTB_CLK_WMASK_VAL 0x01d0131f
#define IAXXX_IO_CTRL_PORTB_CLK_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad PORTB_CLK.
*   0x5: i2s_pcm1_bclk
*   0x6: i2s_cdc_clk
*   0x7: swrm_clk
*   0xc: spi0_cs1_n
*   0xd: spi1_cs1_n
*   0xe: spi2_cs1_n
*   0x10: fo_12
*   0x12: gpio_20
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_PORTB_CLK_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_PORTB_CLK_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_PORTB_CLK_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_PORTB_CLK_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_PORTB_CLK_MUX_SEL_DECL (4:0)

/*
* This field is used to control PCM1_BCLK_AND_SEL for pad PORTB_CLK, a
* separate receiver enable for the PCM1_BCLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTB_CLK_PCM1_BCLK_AND_SEL_MASK 0x00000100
#define IAXXX_IO_CTRL_PORTB_CLK_PCM1_BCLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTB_CLK_PCM1_BCLK_AND_SEL_POS 8
#define IAXXX_IO_CTRL_PORTB_CLK_PCM1_BCLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTB_CLK_PCM1_BCLK_AND_SEL_DECL 8

/*
* This field is used to control PDM1_CLK_AND_SEL for pad PORTB_CLK, a
* separate receiver enable for the PDM1_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTB_CLK_PDM1_CLK_AND_SEL_MASK 0x00000200
#define IAXXX_IO_CTRL_PORTB_CLK_PDM1_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTB_CLK_PDM1_CLK_AND_SEL_POS 9
#define IAXXX_IO_CTRL_PORTB_CLK_PDM1_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTB_CLK_PDM1_CLK_AND_SEL_DECL 9

/*
* Function swrm_clk can be selected through the following pads:
*   COMMD_2
*   COMMF_2
*   PORTB_CLK
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SWRM_CLK_AND_SEL for pad PORTB_CLK, a
* separate receiver enable for the SWRM_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTB_CLK_SWRM_CLK_AND_SEL_MASK 0x00001000
#define IAXXX_IO_CTRL_PORTB_CLK_SWRM_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTB_CLK_SWRM_CLK_AND_SEL_POS 12
#define IAXXX_IO_CTRL_PORTB_CLK_SWRM_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTB_CLK_SWRM_CLK_AND_SEL_DECL 12

/*
* This field is used to control PORTB_CLK2CNR_AND_SEL for pad PORTB_CLK, a
* separate receiver enable for the PORTB_CLK2CNR pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTB_CLK_PORTB_CLK2CNR_AND_SEL_MASK 0x00100000
#define IAXXX_IO_CTRL_PORTB_CLK_PORTB_CLK2CNR_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTB_CLK_PORTB_CLK2CNR_AND_SEL_POS 20
#define IAXXX_IO_CTRL_PORTB_CLK_PORTB_CLK2CNR_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTB_CLK_PORTB_CLK2CNR_AND_SEL_DECL 20

/*
* This field is used to control FI_12_AND_SEL for pad PORTB_CLK, a separate
* receiver enable for the FI_12 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTB_CLK_FI_12_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_PORTB_CLK_FI_12_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTB_CLK_FI_12_AND_SEL_POS 22
#define IAXXX_IO_CTRL_PORTB_CLK_FI_12_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTB_CLK_FI_12_AND_SEL_DECL 22

/*
* This field is used to control GPIO_20_AND_SEL for pad PORTB_CLK, a
* separate receiver enable for the GPIO_20 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTB_CLK_GPIO_20_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_PORTB_CLK_GPIO_20_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_PORTB_CLK_GPIO_20_AND_SEL_POS 23
#define IAXXX_IO_CTRL_PORTB_CLK_GPIO_20_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTB_CLK_GPIO_20_AND_SEL_DECL 23

/*
* Function io2stmr_4_12_20_28 can be selected through the following pads:
*   COMMB_0
*   PORTB_CLK
*   PORTD_CLK
*   CDC_INT_N
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_4_12_20_28_AND_SEL for pad
* PORTB_CLK, a separate receiver enable for the IO2STMR_4_12_20_28 pad
* function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTB_CLK_IO2STMR_4_12_20_28_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_PORTB_CLK_IO2STMR_4_12_20_28_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTB_CLK_IO2STMR_4_12_20_28_AND_SEL_POS 24
#define IAXXX_IO_CTRL_PORTB_CLK_IO2STMR_4_12_20_28_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTB_CLK_IO2STMR_4_12_20_28_AND_SEL_DECL 24

/*** IO_CTRL_PORTB_DI (0x500220b8) ***/
/* This register is used for pad PORTB_DI's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_PORTB_DI_ADDR (0x500220b8)
#define IAXXX_IO_CTRL_PORTB_DI_MASK_VAL 0x01c0131f
#define IAXXX_IO_CTRL_PORTB_DI_RMASK_VAL 0x01c0131f
#define IAXXX_IO_CTRL_PORTB_DI_WMASK_VAL 0x01c0131f
#define IAXXX_IO_CTRL_PORTB_DI_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad PORTB_DI.
*   0x7: swrm_data
*   0xc: spi0_cs1_n
*   0xd: spi1_cs1_n
*   0xe: spi2_cs1_n
*   0x12: gpio_22
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_PORTB_DI_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_PORTB_DI_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_PORTB_DI_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_PORTB_DI_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_PORTB_DI_MUX_SEL_DECL (4:0)

/*
* This field is used to control PCM1_DR_AND_SEL for pad PORTB_DI, a separate
* receiver enable for the PCM1_DR pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTB_DI_PCM1_DR_AND_SEL_MASK 0x00000100
#define IAXXX_IO_CTRL_PORTB_DI_PCM1_DR_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTB_DI_PCM1_DR_AND_SEL_POS 8
#define IAXXX_IO_CTRL_PORTB_DI_PCM1_DR_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTB_DI_PCM1_DR_AND_SEL_DECL 8

/*
* This field is used to control PDM1_DI1_AND_SEL for pad PORTB_DI, a
* separate receiver enable for the PDM1_DI1 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTB_DI_PDM1_DI1_AND_SEL_MASK 0x00000200
#define IAXXX_IO_CTRL_PORTB_DI_PDM1_DI1_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTB_DI_PDM1_DI1_AND_SEL_POS 9
#define IAXXX_IO_CTRL_PORTB_DI_PDM1_DI1_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTB_DI_PDM1_DI1_AND_SEL_DECL 9

/*
* Function swrm_data can be selected through the following pads:
*   COMMD_3
*   COMMF_3
*   PORTB_DI
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SWRM_DATA_AND_SEL for pad PORTB_DI, a
* separate receiver enable for the SWRM_DATA pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTB_DI_SWRM_DATA_AND_SEL_MASK 0x00001000
#define IAXXX_IO_CTRL_PORTB_DI_SWRM_DATA_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTB_DI_SWRM_DATA_AND_SEL_POS 12
#define IAXXX_IO_CTRL_PORTB_DI_SWRM_DATA_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTB_DI_SWRM_DATA_AND_SEL_DECL 12

/*
* This field is used to control FI_14_AND_SEL for pad PORTB_DI, a separate
* receiver enable for the FI_14 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTB_DI_FI_14_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_PORTB_DI_FI_14_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTB_DI_FI_14_AND_SEL_POS 22
#define IAXXX_IO_CTRL_PORTB_DI_FI_14_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTB_DI_FI_14_AND_SEL_DECL 22

/*
* This field is used to control GPIO_22_AND_SEL for pad PORTB_DI, a separate
* receiver enable for the GPIO_22 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTB_DI_GPIO_22_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_PORTB_DI_GPIO_22_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_PORTB_DI_GPIO_22_AND_SEL_POS 23
#define IAXXX_IO_CTRL_PORTB_DI_GPIO_22_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTB_DI_GPIO_22_AND_SEL_DECL 23

/*
* Function io2stmr_6_14_22_30 can be selected through the following pads:
*   COMMB_2
*   PORTB_DI
*   PORTD_DI
*   CDC_I2C_SDA_WR
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_6_14_22_30_AND_SEL for pad PORTB_DI,
* a separate receiver enable for the IO2STMR_6_14_22_30 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTB_DI_IO2STMR_6_14_22_30_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_PORTB_DI_IO2STMR_6_14_22_30_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTB_DI_IO2STMR_6_14_22_30_AND_SEL_POS 24
#define IAXXX_IO_CTRL_PORTB_DI_IO2STMR_6_14_22_30_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTB_DI_IO2STMR_6_14_22_30_AND_SEL_DECL 24

/*** IO_CTRL_PORTB_DO (0x500220bc) ***/
/* This register is used for pad PORTB_DO's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_PORTB_DO_ADDR (0x500220bc)
#define IAXXX_IO_CTRL_PORTB_DO_MASK_VAL 0x01c0001f
#define IAXXX_IO_CTRL_PORTB_DO_RMASK_VAL 0x01c0001f
#define IAXXX_IO_CTRL_PORTB_DO_WMASK_VAL 0x01c0001f
#define IAXXX_IO_CTRL_PORTB_DO_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad PORTB_DO.
*   0x3: pcm1_dt
*   0x4: pdm_do
*   0xc: spi0_cs3_n
*   0xd: spi1_cs3_n
*   0xe: spi2_cs3_n
*   0x12: gpio_23
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_PORTB_DO_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_PORTB_DO_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_PORTB_DO_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_PORTB_DO_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_PORTB_DO_MUX_SEL_DECL (4:0)

/*
* This field is used to control FI_15_AND_SEL for pad PORTB_DO, a separate
* receiver enable for the FI_15 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTB_DO_FI_15_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_PORTB_DO_FI_15_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTB_DO_FI_15_AND_SEL_POS 22
#define IAXXX_IO_CTRL_PORTB_DO_FI_15_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTB_DO_FI_15_AND_SEL_DECL 22

/*
* This field is used to control GPIO_23_AND_SEL for pad PORTB_DO, a separate
* receiver enable for the GPIO_23 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTB_DO_GPIO_23_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_PORTB_DO_GPIO_23_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_PORTB_DO_GPIO_23_AND_SEL_POS 23
#define IAXXX_IO_CTRL_PORTB_DO_GPIO_23_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTB_DO_GPIO_23_AND_SEL_DECL 23

/*
* Function io2stmr_7_15_23_31 can be selected through the following pads:
*   COMMB_3
*   PORTB_DO
*   PORTD_DO
*   CDC_I2C_SDA_RD
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_7_15_23_31_AND_SEL for pad PORTB_DO,
* a separate receiver enable for the IO2STMR_7_15_23_31 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTB_DO_IO2STMR_7_15_23_31_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_PORTB_DO_IO2STMR_7_15_23_31_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTB_DO_IO2STMR_7_15_23_31_AND_SEL_POS 24
#define IAXXX_IO_CTRL_PORTB_DO_IO2STMR_7_15_23_31_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTB_DO_IO2STMR_7_15_23_31_AND_SEL_DECL 24

/*** IO_CTRL_PORTB_FS (0x500220c0) ***/
/* This register is used for pad PORTB_FS's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_PORTB_FS_ADDR (0x500220c0)
#define IAXXX_IO_CTRL_PORTB_FS_MASK_VAL 0x01c0031f
#define IAXXX_IO_CTRL_PORTB_FS_RMASK_VAL 0x01c0031f
#define IAXXX_IO_CTRL_PORTB_FS_WMASK_VAL 0x01c0031f
#define IAXXX_IO_CTRL_PORTB_FS_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad PORTB_FS.
*   0x5: i2s_pcm1_fs
*   0xc: spi0_cs2_n
*   0xd: spi1_cs2_n
*   0xe: spi2_cs2_n
*   0x10: fo_13
*   0x12: gpio_21
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_PORTB_FS_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_PORTB_FS_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_PORTB_FS_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_PORTB_FS_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_PORTB_FS_MUX_SEL_DECL (4:0)

/*
* This field is used to control PCM1_FS_AND_SEL for pad PORTB_FS, a separate
* receiver enable for the PCM1_FS pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTB_FS_PCM1_FS_AND_SEL_MASK 0x00000100
#define IAXXX_IO_CTRL_PORTB_FS_PCM1_FS_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTB_FS_PCM1_FS_AND_SEL_POS 8
#define IAXXX_IO_CTRL_PORTB_FS_PCM1_FS_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTB_FS_PCM1_FS_AND_SEL_DECL 8

/*
* This field is used to control PDM1_DI0_AND_SEL for pad PORTB_FS, a
* separate receiver enable for the PDM1_DI0 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTB_FS_PDM1_DI0_AND_SEL_MASK 0x00000200
#define IAXXX_IO_CTRL_PORTB_FS_PDM1_DI0_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTB_FS_PDM1_DI0_AND_SEL_POS 9
#define IAXXX_IO_CTRL_PORTB_FS_PDM1_DI0_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTB_FS_PDM1_DI0_AND_SEL_DECL 9

/*
* This field is used to control FI_13_AND_SEL for pad PORTB_FS, a separate
* receiver enable for the FI_13 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTB_FS_FI_13_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_PORTB_FS_FI_13_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTB_FS_FI_13_AND_SEL_POS 22
#define IAXXX_IO_CTRL_PORTB_FS_FI_13_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTB_FS_FI_13_AND_SEL_DECL 22

/*
* This field is used to control GPIO_21_AND_SEL for pad PORTB_FS, a separate
* receiver enable for the GPIO_21 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTB_FS_GPIO_21_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_PORTB_FS_GPIO_21_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_PORTB_FS_GPIO_21_AND_SEL_POS 23
#define IAXXX_IO_CTRL_PORTB_FS_GPIO_21_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTB_FS_GPIO_21_AND_SEL_DECL 23

/*
* Function io2stmr_5_13_21_29 can be selected through the following pads:
*   COMMB_1
*   PORTB_FS
*   PORTD_FS
*   CDC_I2C_SCL
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_5_13_21_29_AND_SEL for pad PORTB_FS,
* a separate receiver enable for the IO2STMR_5_13_21_29 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTB_FS_IO2STMR_5_13_21_29_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_PORTB_FS_IO2STMR_5_13_21_29_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTB_FS_IO2STMR_5_13_21_29_AND_SEL_POS 24
#define IAXXX_IO_CTRL_PORTB_FS_IO2STMR_5_13_21_29_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTB_FS_IO2STMR_5_13_21_29_AND_SEL_DECL 24

/*** IO_CTRL_PORTC_CLK (0x500220c4) ***/
/* This register is used for pad PORTC_CLK's IO function control.Refer */
/* <a href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_PORTC_CLK_ADDR (0x500220c4)
#define IAXXX_IO_CTRL_PORTC_CLK_MASK_VAL 0x01d0031f
#define IAXXX_IO_CTRL_PORTC_CLK_RMASK_VAL 0x01d0031f
#define IAXXX_IO_CTRL_PORTC_CLK_WMASK_VAL 0x01d0031f
#define IAXXX_IO_CTRL_PORTC_CLK_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad PORTC_CLK.
*   0x5: i2s_pcm2_bclk
*   0x6: i2s_cdc_clk
*   0xc: spi0_cs1_n
*   0xd: spi1_cs1_n
*   0xe: spi2_cs1_n
*   0x12: gpio_24
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_PORTC_CLK_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_PORTC_CLK_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_PORTC_CLK_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_PORTC_CLK_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_PORTC_CLK_MUX_SEL_DECL (4:0)

/*
* This field is used to control PCM2_BCLK_AND_SEL for pad PORTC_CLK, a
* separate receiver enable for the PCM2_BCLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTC_CLK_PCM2_BCLK_AND_SEL_MASK 0x00000100
#define IAXXX_IO_CTRL_PORTC_CLK_PCM2_BCLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTC_CLK_PCM2_BCLK_AND_SEL_POS 8
#define IAXXX_IO_CTRL_PORTC_CLK_PCM2_BCLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTC_CLK_PCM2_BCLK_AND_SEL_DECL 8

/*
* This field is used to control PDM0_CLK_AND_SEL for pad PORTC_CLK, a
* separate receiver enable for the PDM0_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTC_CLK_PDM0_CLK_AND_SEL_MASK 0x00000200
#define IAXXX_IO_CTRL_PORTC_CLK_PDM0_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTC_CLK_PDM0_CLK_AND_SEL_POS 9
#define IAXXX_IO_CTRL_PORTC_CLK_PDM0_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTC_CLK_PDM0_CLK_AND_SEL_DECL 9

/*
* This field is used to control PORTC_CLK2CNR_AND_SEL for pad PORTC_CLK, a
* separate receiver enable for the PORTC_CLK2CNR pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTC_CLK_PORTC_CLK2CNR_AND_SEL_MASK 0x00100000
#define IAXXX_IO_CTRL_PORTC_CLK_PORTC_CLK2CNR_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTC_CLK_PORTC_CLK2CNR_AND_SEL_POS 20
#define IAXXX_IO_CTRL_PORTC_CLK_PORTC_CLK2CNR_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTC_CLK_PORTC_CLK2CNR_AND_SEL_DECL 20

/*
* This field is used to control FI_16_AND_SEL for pad PORTC_CLK, a separate
* receiver enable for the FI_16 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTC_CLK_FI_16_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_PORTC_CLK_FI_16_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTC_CLK_FI_16_AND_SEL_POS 22
#define IAXXX_IO_CTRL_PORTC_CLK_FI_16_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTC_CLK_FI_16_AND_SEL_DECL 22

/*
* This field is used to control GPIO_24_AND_SEL for pad PORTC_CLK, a
* separate receiver enable for the GPIO_24 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTC_CLK_GPIO_24_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_PORTC_CLK_GPIO_24_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_PORTC_CLK_GPIO_24_AND_SEL_POS 23
#define IAXXX_IO_CTRL_PORTC_CLK_GPIO_24_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTC_CLK_GPIO_24_AND_SEL_DECL 23

/*
* Function io2stmr_0_8_16_24 can be selected through the following pads:
*   COMMA_2
*   PORTA_CLK
*   PORTC_CLK
*   PORTE_CLK
*   CDC_PWRON
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_0_8_16_24_AND_SEL for pad PORTC_CLK,
* a separate receiver enable for the IO2STMR_0_8_16_24 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTC_CLK_IO2STMR_0_8_16_24_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_PORTC_CLK_IO2STMR_0_8_16_24_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTC_CLK_IO2STMR_0_8_16_24_AND_SEL_POS 24
#define IAXXX_IO_CTRL_PORTC_CLK_IO2STMR_0_8_16_24_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTC_CLK_IO2STMR_0_8_16_24_AND_SEL_DECL 24

/*** IO_CTRL_PORTC_DI (0x500220c8) ***/
/* This register is used for pad PORTC_DI's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_PORTC_DI_ADDR (0x500220c8)
#define IAXXX_IO_CTRL_PORTC_DI_MASK_VAL 0x01c0031f
#define IAXXX_IO_CTRL_PORTC_DI_RMASK_VAL 0x01c0031f
#define IAXXX_IO_CTRL_PORTC_DI_WMASK_VAL 0x01c0031f
#define IAXXX_IO_CTRL_PORTC_DI_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad PORTC_DI.
*   0xc: spi0_cs1_n
*   0xd: spi1_cs1_n
*   0xe: spi2_cs1_n
*   0x12: gpio_26
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_PORTC_DI_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_PORTC_DI_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_PORTC_DI_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_PORTC_DI_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_PORTC_DI_MUX_SEL_DECL (4:0)

/*
* This field is used to control PCM2_DR_AND_SEL for pad PORTC_DI, a separate
* receiver enable for the PCM2_DR pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTC_DI_PCM2_DR_AND_SEL_MASK 0x00000100
#define IAXXX_IO_CTRL_PORTC_DI_PCM2_DR_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTC_DI_PCM2_DR_AND_SEL_POS 8
#define IAXXX_IO_CTRL_PORTC_DI_PCM2_DR_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTC_DI_PCM2_DR_AND_SEL_DECL 8

/*
* This field is used to control PDM0_DI1_AND_SEL for pad PORTC_DI, a
* separate receiver enable for the PDM0_DI1 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTC_DI_PDM0_DI1_AND_SEL_MASK 0x00000200
#define IAXXX_IO_CTRL_PORTC_DI_PDM0_DI1_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTC_DI_PDM0_DI1_AND_SEL_POS 9
#define IAXXX_IO_CTRL_PORTC_DI_PDM0_DI1_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTC_DI_PDM0_DI1_AND_SEL_DECL 9

/*
* This field is used to control FI_18_AND_SEL for pad PORTC_DI, a separate
* receiver enable for the FI_18 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTC_DI_FI_18_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_PORTC_DI_FI_18_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTC_DI_FI_18_AND_SEL_POS 22
#define IAXXX_IO_CTRL_PORTC_DI_FI_18_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTC_DI_FI_18_AND_SEL_DECL 22

/*
* This field is used to control GPIO_26_AND_SEL for pad PORTC_DI, a separate
* receiver enable for the GPIO_26 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTC_DI_GPIO_26_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_PORTC_DI_GPIO_26_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_PORTC_DI_GPIO_26_AND_SEL_POS 23
#define IAXXX_IO_CTRL_PORTC_DI_GPIO_26_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTC_DI_GPIO_26_AND_SEL_DECL 23

/*
* Function io2stmr_2_10_18_26 can be selected through the following pads:
*   COMMA_0
*   INTR_EVNT
*   PORTA_DI
*   PORTC_DI
*   PORTE_DI
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_2_10_18_26_AND_SEL for pad PORTC_DI,
* a separate receiver enable for the IO2STMR_2_10_18_26 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTC_DI_IO2STMR_2_10_18_26_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_PORTC_DI_IO2STMR_2_10_18_26_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTC_DI_IO2STMR_2_10_18_26_AND_SEL_POS 24
#define IAXXX_IO_CTRL_PORTC_DI_IO2STMR_2_10_18_26_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTC_DI_IO2STMR_2_10_18_26_AND_SEL_DECL 24

/*** IO_CTRL_PORTC_DO (0x500220cc) ***/
/* This register is used for pad PORTC_DO's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_PORTC_DO_ADDR (0x500220cc)
#define IAXXX_IO_CTRL_PORTC_DO_MASK_VAL 0x01c0001f
#define IAXXX_IO_CTRL_PORTC_DO_RMASK_VAL 0x01c0001f
#define IAXXX_IO_CTRL_PORTC_DO_WMASK_VAL 0x01c0001f
#define IAXXX_IO_CTRL_PORTC_DO_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad PORTC_DO.
*   0x3: pcm2_dt
*   0x4: pdm_do
*   0xc: spi0_cs3_n
*   0xd: spi1_cs3_n
*   0xe: spi2_cs3_n
*   0x12: gpio_27
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_PORTC_DO_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_PORTC_DO_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_PORTC_DO_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_PORTC_DO_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_PORTC_DO_MUX_SEL_DECL (4:0)

/*
* This field is used to control FI_19_AND_SEL for pad PORTC_DO, a separate
* receiver enable for the FI_19 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTC_DO_FI_19_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_PORTC_DO_FI_19_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTC_DO_FI_19_AND_SEL_POS 22
#define IAXXX_IO_CTRL_PORTC_DO_FI_19_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTC_DO_FI_19_AND_SEL_DECL 22

/*
* This field is used to control GPIO_27_AND_SEL for pad PORTC_DO, a separate
* receiver enable for the GPIO_27 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTC_DO_GPIO_27_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_PORTC_DO_GPIO_27_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_PORTC_DO_GPIO_27_AND_SEL_POS 23
#define IAXXX_IO_CTRL_PORTC_DO_GPIO_27_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTC_DO_GPIO_27_AND_SEL_DECL 23

/*
* Function io2stmr_3_11_19_27 can be selected through the following pads:
*   COMMA_1
*   PORTA_DO
*   PORTC_DO
*   PORTE_DO
*   CDC_RESET_N
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_3_11_19_27_AND_SEL for pad PORTC_DO,
* a separate receiver enable for the IO2STMR_3_11_19_27 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTC_DO_IO2STMR_3_11_19_27_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_PORTC_DO_IO2STMR_3_11_19_27_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTC_DO_IO2STMR_3_11_19_27_AND_SEL_POS 24
#define IAXXX_IO_CTRL_PORTC_DO_IO2STMR_3_11_19_27_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTC_DO_IO2STMR_3_11_19_27_AND_SEL_DECL 24

/*** IO_CTRL_PORTC_FS (0x500220d0) ***/
/* This register is used for pad PORTC_FS's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_PORTC_FS_ADDR (0x500220d0)
#define IAXXX_IO_CTRL_PORTC_FS_MASK_VAL 0x01c0031f
#define IAXXX_IO_CTRL_PORTC_FS_RMASK_VAL 0x01c0031f
#define IAXXX_IO_CTRL_PORTC_FS_WMASK_VAL 0x01c0031f
#define IAXXX_IO_CTRL_PORTC_FS_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad PORTC_FS.
*   0x5: i2s_pcm2_fs
*   0xc: spi0_cs2_n
*   0xd: spi1_cs2_n
*   0xe: spi2_cs2_n
*   0x12: gpio_25
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_PORTC_FS_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_PORTC_FS_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_PORTC_FS_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_PORTC_FS_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_PORTC_FS_MUX_SEL_DECL (4:0)

/*
* This field is used to control PCM2_FS_AND_SEL for pad PORTC_FS, a separate
* receiver enable for the PCM2_FS pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTC_FS_PCM2_FS_AND_SEL_MASK 0x00000100
#define IAXXX_IO_CTRL_PORTC_FS_PCM2_FS_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTC_FS_PCM2_FS_AND_SEL_POS 8
#define IAXXX_IO_CTRL_PORTC_FS_PCM2_FS_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTC_FS_PCM2_FS_AND_SEL_DECL 8

/*
* This field is used to control PDM0_DI0_AND_SEL for pad PORTC_FS, a
* separate receiver enable for the PDM0_DI0 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTC_FS_PDM0_DI0_AND_SEL_MASK 0x00000200
#define IAXXX_IO_CTRL_PORTC_FS_PDM0_DI0_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTC_FS_PDM0_DI0_AND_SEL_POS 9
#define IAXXX_IO_CTRL_PORTC_FS_PDM0_DI0_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTC_FS_PDM0_DI0_AND_SEL_DECL 9

/*
* This field is used to control FI_17_AND_SEL for pad PORTC_FS, a separate
* receiver enable for the FI_17 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTC_FS_FI_17_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_PORTC_FS_FI_17_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTC_FS_FI_17_AND_SEL_POS 22
#define IAXXX_IO_CTRL_PORTC_FS_FI_17_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTC_FS_FI_17_AND_SEL_DECL 22

/*
* This field is used to control GPIO_25_AND_SEL for pad PORTC_FS, a separate
* receiver enable for the GPIO_25 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTC_FS_GPIO_25_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_PORTC_FS_GPIO_25_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_PORTC_FS_GPIO_25_AND_SEL_POS 23
#define IAXXX_IO_CTRL_PORTC_FS_GPIO_25_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTC_FS_GPIO_25_AND_SEL_DECL 23

/*
* Function io2stmr_1_9_17_25 can be selected through the following pads:
*   COMMA_3
*   INTR_API
*   PORTA_FS
*   PORTC_FS
*   PORTE_FS
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_1_9_17_25_AND_SEL for pad PORTC_FS,
* a separate receiver enable for the IO2STMR_1_9_17_25 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTC_FS_IO2STMR_1_9_17_25_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_PORTC_FS_IO2STMR_1_9_17_25_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTC_FS_IO2STMR_1_9_17_25_AND_SEL_POS 24
#define IAXXX_IO_CTRL_PORTC_FS_IO2STMR_1_9_17_25_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTC_FS_IO2STMR_1_9_17_25_AND_SEL_DECL 24

/*** IO_CTRL_PORTD_CLK (0x500220d4) ***/
/* This register is used for pad PORTD_CLK's IO function control.Refer */
/* <a href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_PORTD_CLK_ADDR (0x500220d4)
#define IAXXX_IO_CTRL_PORTD_CLK_MASK_VAL 0x01d1e11f
#define IAXXX_IO_CTRL_PORTD_CLK_RMASK_VAL 0x01d1e11f
#define IAXXX_IO_CTRL_PORTD_CLK_WMASK_VAL 0x01d1e11f
#define IAXXX_IO_CTRL_PORTD_CLK_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad PORTD_CLK.
*   0x5: i2s_pcm3_bclk
*   0x6: i2s_cdc_clk
*   0x9: i2c0_clk
*   0xc: spi0_cs1_n
*   0xd: spi1_cs1_n
*   0xe: spi2_cs1_n
*   0x10: fo_8
*   0x12: gpio_28
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_PORTD_CLK_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_PORTD_CLK_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_PORTD_CLK_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_PORTD_CLK_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_PORTD_CLK_MUX_SEL_DECL (4:0)

/*
* This field is used to control PCM3_BCLK_AND_SEL for pad PORTD_CLK, a
* separate receiver enable for the PCM3_BCLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTD_CLK_PCM3_BCLK_AND_SEL_MASK 0x00000100
#define IAXXX_IO_CTRL_PORTD_CLK_PCM3_BCLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTD_CLK_PCM3_BCLK_AND_SEL_POS 8
#define IAXXX_IO_CTRL_PORTD_CLK_PCM3_BCLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTD_CLK_PCM3_BCLK_AND_SEL_DECL 8

/*
* This field is used to control CDC1_CLK_AND_SEL for pad PORTD_CLK, a
* separate receiver enable for the CDC1_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTD_CLK_CDC1_CLK_AND_SEL_MASK 0x00002000
#define IAXXX_IO_CTRL_PORTD_CLK_CDC1_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTD_CLK_CDC1_CLK_AND_SEL_POS 13
#define IAXXX_IO_CTRL_PORTD_CLK_CDC1_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTD_CLK_CDC1_CLK_AND_SEL_DECL 13

/*
* Function i2c0_clk can be selected through the following pads:
*   COMMA_0
*   COMMD_0
*   PORTD_CLK
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C0_CLK_AND_SEL for pad PORTD_CLK, a
* separate receiver enable for the I2C0_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTD_CLK_I2C0_CLK_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_PORTD_CLK_I2C0_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTD_CLK_I2C0_CLK_AND_SEL_POS 14
#define IAXXX_IO_CTRL_PORTD_CLK_I2C0_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTD_CLK_I2C0_CLK_AND_SEL_DECL 14

/*
* Function uart1_sin can be selected through the following pads:
*   COMMB_2
*   COMMD_2
*   PORTD_CLK
*   PORTE_CLK
*   CDC_INT_N
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control UART1_SIN_AND_SEL for pad PORTD_CLK, a
* separate receiver enable for the UART1_SIN pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTD_CLK_UART1_SIN_AND_SEL_MASK 0x00008000
#define IAXXX_IO_CTRL_PORTD_CLK_UART1_SIN_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTD_CLK_UART1_SIN_AND_SEL_POS 15
#define IAXXX_IO_CTRL_PORTD_CLK_UART1_SIN_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTD_CLK_UART1_SIN_AND_SEL_DECL 15

/*
* Function spi0_rxd can be selected through the following pads:
*   COMMA_0
*   COMMD_0
*   PORTD_CLK
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SPI0_RXD_AND_SEL for pad PORTD_CLK, a
* separate receiver enable for the SPI0_RXD pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTD_CLK_SPI0_RXD_AND_SEL_MASK 0x00010000
#define IAXXX_IO_CTRL_PORTD_CLK_SPI0_RXD_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTD_CLK_SPI0_RXD_AND_SEL_POS 16
#define IAXXX_IO_CTRL_PORTD_CLK_SPI0_RXD_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTD_CLK_SPI0_RXD_AND_SEL_DECL 16

/*
* This field is used to control PORTD_CLK2CNR_AND_SEL for pad PORTD_CLK, a
* separate receiver enable for the PORTD_CLK2CNR pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTD_CLK_PORTD_CLK2CNR_AND_SEL_MASK 0x00100000
#define IAXXX_IO_CTRL_PORTD_CLK_PORTD_CLK2CNR_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTD_CLK_PORTD_CLK2CNR_AND_SEL_POS 20
#define IAXXX_IO_CTRL_PORTD_CLK_PORTD_CLK2CNR_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTD_CLK_PORTD_CLK2CNR_AND_SEL_DECL 20

/*
* This field is used to control FI_20_AND_SEL for pad PORTD_CLK, a separate
* receiver enable for the FI_20 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTD_CLK_FI_20_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_PORTD_CLK_FI_20_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTD_CLK_FI_20_AND_SEL_POS 22
#define IAXXX_IO_CTRL_PORTD_CLK_FI_20_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTD_CLK_FI_20_AND_SEL_DECL 22

/*
* This field is used to control GPIO_28_AND_SEL for pad PORTD_CLK, a
* separate receiver enable for the GPIO_28 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTD_CLK_GPIO_28_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_PORTD_CLK_GPIO_28_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_PORTD_CLK_GPIO_28_AND_SEL_POS 23
#define IAXXX_IO_CTRL_PORTD_CLK_GPIO_28_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTD_CLK_GPIO_28_AND_SEL_DECL 23

/*
* Function io2stmr_4_12_20_28 can be selected through the following pads:
*   COMMB_0
*   PORTB_CLK
*   PORTD_CLK
*   CDC_INT_N
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_4_12_20_28_AND_SEL for pad
* PORTD_CLK, a separate receiver enable for the IO2STMR_4_12_20_28 pad
* function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTD_CLK_IO2STMR_4_12_20_28_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_PORTD_CLK_IO2STMR_4_12_20_28_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTD_CLK_IO2STMR_4_12_20_28_AND_SEL_POS 24
#define IAXXX_IO_CTRL_PORTD_CLK_IO2STMR_4_12_20_28_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTD_CLK_IO2STMR_4_12_20_28_AND_SEL_DECL 24

/*** IO_CTRL_PORTD_DI (0x500220d8) ***/
/* This register is used for pad PORTD_DI's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_PORTD_DI_ADDR (0x500220d8)
#define IAXXX_IO_CTRL_PORTD_DI_MASK_VAL 0x01c1a11f
#define IAXXX_IO_CTRL_PORTD_DI_RMASK_VAL 0x01c1a11f
#define IAXXX_IO_CTRL_PORTD_DI_WMASK_VAL 0x01c1a11f
#define IAXXX_IO_CTRL_PORTD_DI_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad PORTD_DI.
*   0xb: spi0_cs0_n
*   0xc: spi0_cs1_n
*   0xd: spi1_cs1_n
*   0xe: spi2_cs1_n
*   0x10: fo_10
*   0x12: gpio_30
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_PORTD_DI_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_PORTD_DI_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_PORTD_DI_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_PORTD_DI_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_PORTD_DI_MUX_SEL_DECL (4:0)

/*
* This field is used to control PCM3_DR_AND_SEL for pad PORTD_DI, a separate
* receiver enable for the PCM3_DR pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTD_DI_PCM3_DR_AND_SEL_MASK 0x00000100
#define IAXXX_IO_CTRL_PORTD_DI_PCM3_DR_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTD_DI_PCM3_DR_AND_SEL_POS 8
#define IAXXX_IO_CTRL_PORTD_DI_PCM3_DR_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTD_DI_PCM3_DR_AND_SEL_DECL 8

/*
* This field is used to control CDC_ADC_3_AND_SEL for pad PORTD_DI, a
* separate receiver enable for the CDC_ADC_3 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTD_DI_CDC_ADC_3_AND_SEL_MASK 0x00002000
#define IAXXX_IO_CTRL_PORTD_DI_CDC_ADC_3_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTD_DI_CDC_ADC_3_AND_SEL_POS 13
#define IAXXX_IO_CTRL_PORTD_DI_CDC_ADC_3_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTD_DI_CDC_ADC_3_AND_SEL_DECL 13

/*
* Function uart0_sin can be selected through the following pads:
*   COMMA_2
*   COMMC_2
*   COMMD_0
*   PORTD_DI
*   PORTE_DI
*   CDC_PWRON
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control UART0_SIN_AND_SEL for pad PORTD_DI, a
* separate receiver enable for the UART0_SIN pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTD_DI_UART0_SIN_AND_SEL_MASK 0x00008000
#define IAXXX_IO_CTRL_PORTD_DI_UART0_SIN_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTD_DI_UART0_SIN_AND_SEL_POS 15
#define IAXXX_IO_CTRL_PORTD_DI_UART0_SIN_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTD_DI_UART0_SIN_AND_SEL_DECL 15

/*
* Function spi0_cs0_n can be selected through the following pads:
*   COMMA_2
*   COMMD_2
*   PORTD_DI
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SPI0_CS0_N_AND_SEL for pad PORTD_DI, a
* separate receiver enable for the SPI0_CS0_N pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTD_DI_SPI0_CS0_N_AND_SEL_MASK 0x00010000
#define IAXXX_IO_CTRL_PORTD_DI_SPI0_CS0_N_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTD_DI_SPI0_CS0_N_AND_SEL_POS 16
#define IAXXX_IO_CTRL_PORTD_DI_SPI0_CS0_N_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTD_DI_SPI0_CS0_N_AND_SEL_DECL 16

/*
* This field is used to control FI_22_AND_SEL for pad PORTD_DI, a separate
* receiver enable for the FI_22 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTD_DI_FI_22_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_PORTD_DI_FI_22_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTD_DI_FI_22_AND_SEL_POS 22
#define IAXXX_IO_CTRL_PORTD_DI_FI_22_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTD_DI_FI_22_AND_SEL_DECL 22

/*
* This field is used to control GPIO_30_AND_SEL for pad PORTD_DI, a separate
* receiver enable for the GPIO_30 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTD_DI_GPIO_30_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_PORTD_DI_GPIO_30_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_PORTD_DI_GPIO_30_AND_SEL_POS 23
#define IAXXX_IO_CTRL_PORTD_DI_GPIO_30_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTD_DI_GPIO_30_AND_SEL_DECL 23

/*
* Function io2stmr_6_14_22_30 can be selected through the following pads:
*   COMMB_2
*   PORTB_DI
*   PORTD_DI
*   CDC_I2C_SDA_WR
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_6_14_22_30_AND_SEL for pad PORTD_DI,
* a separate receiver enable for the IO2STMR_6_14_22_30 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTD_DI_IO2STMR_6_14_22_30_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_PORTD_DI_IO2STMR_6_14_22_30_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTD_DI_IO2STMR_6_14_22_30_AND_SEL_POS 24
#define IAXXX_IO_CTRL_PORTD_DI_IO2STMR_6_14_22_30_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTD_DI_IO2STMR_6_14_22_30_AND_SEL_DECL 24

/*** IO_CTRL_PORTD_DO (0x500220dc) ***/
/* This register is used for pad PORTD_DO's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_PORTD_DO_ADDR (0x500220dc)
#define IAXXX_IO_CTRL_PORTD_DO_MASK_VAL 0x01c0001f
#define IAXXX_IO_CTRL_PORTD_DO_RMASK_VAL 0x01c0001f
#define IAXXX_IO_CTRL_PORTD_DO_WMASK_VAL 0x01c0001f
#define IAXXX_IO_CTRL_PORTD_DO_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad PORTD_DO.
*   0x3: pcm3_dt
*   0xa: uart0_sout
*   0xb: spi0_txd
*   0xc: spi0_cs3_n
*   0xd: spi1_cs3_n
*   0xe: spi2_cs3_n
*   0x10: fo_11
*   0x12: gpio_31
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_PORTD_DO_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_PORTD_DO_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_PORTD_DO_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_PORTD_DO_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_PORTD_DO_MUX_SEL_DECL (4:0)

/*
* This field is used to control FI_23_AND_SEL for pad PORTD_DO, a separate
* receiver enable for the FI_23 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTD_DO_FI_23_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_PORTD_DO_FI_23_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTD_DO_FI_23_AND_SEL_POS 22
#define IAXXX_IO_CTRL_PORTD_DO_FI_23_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTD_DO_FI_23_AND_SEL_DECL 22

/*
* This field is used to control GPIO_31_AND_SEL for pad PORTD_DO, a separate
* receiver enable for the GPIO_31 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTD_DO_GPIO_31_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_PORTD_DO_GPIO_31_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_PORTD_DO_GPIO_31_AND_SEL_POS 23
#define IAXXX_IO_CTRL_PORTD_DO_GPIO_31_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTD_DO_GPIO_31_AND_SEL_DECL 23

/*
* Function io2stmr_7_15_23_31 can be selected through the following pads:
*   COMMB_3
*   PORTB_DO
*   PORTD_DO
*   CDC_I2C_SDA_RD
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_7_15_23_31_AND_SEL for pad PORTD_DO,
* a separate receiver enable for the IO2STMR_7_15_23_31 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTD_DO_IO2STMR_7_15_23_31_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_PORTD_DO_IO2STMR_7_15_23_31_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTD_DO_IO2STMR_7_15_23_31_AND_SEL_POS 24
#define IAXXX_IO_CTRL_PORTD_DO_IO2STMR_7_15_23_31_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTD_DO_IO2STMR_7_15_23_31_AND_SEL_DECL 24

/*** IO_CTRL_PORTD_FS (0x500220e0) ***/
/* This register is used for pad PORTD_FS's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_PORTD_FS_ADDR (0x500220e0)
#define IAXXX_IO_CTRL_PORTD_FS_MASK_VAL 0x01c1411f
#define IAXXX_IO_CTRL_PORTD_FS_RMASK_VAL 0x01c1411f
#define IAXXX_IO_CTRL_PORTD_FS_WMASK_VAL 0x01c1411f
#define IAXXX_IO_CTRL_PORTD_FS_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad PORTD_FS.
*   0x5: i2s_pcm3_fs
*   0x9: i2c0_data
*   0xa: uart1_sout
*   0xb: spi0_clk
*   0xc: spi0_cs2_n
*   0xd: spi1_cs2_n
*   0xe: spi2_cs2_n
*   0x10: fo_9
*   0x12: gpio_29
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_PORTD_FS_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_PORTD_FS_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_PORTD_FS_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_PORTD_FS_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_PORTD_FS_MUX_SEL_DECL (4:0)

/*
* This field is used to control PCM3_FS_AND_SEL for pad PORTD_FS, a separate
* receiver enable for the PCM3_FS pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTD_FS_PCM3_FS_AND_SEL_MASK 0x00000100
#define IAXXX_IO_CTRL_PORTD_FS_PCM3_FS_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTD_FS_PCM3_FS_AND_SEL_POS 8
#define IAXXX_IO_CTRL_PORTD_FS_PCM3_FS_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTD_FS_PCM3_FS_AND_SEL_DECL 8

/*
* Function i2c0_data can be selected through the following pads:
*   COMMA_1
*   COMMD_1
*   PORTD_FS
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C0_DATA_AND_SEL for pad PORTD_FS, a
* separate receiver enable for the I2C0_DATA pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTD_FS_I2C0_DATA_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_PORTD_FS_I2C0_DATA_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTD_FS_I2C0_DATA_AND_SEL_POS 14
#define IAXXX_IO_CTRL_PORTD_FS_I2C0_DATA_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTD_FS_I2C0_DATA_AND_SEL_DECL 14

/*
* Function spi0_clk can be selected through the following pads:
*   COMMA_1
*   COMMD_1
*   PORTD_FS
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SPI0_CLK_AND_SEL for pad PORTD_FS, a
* separate receiver enable for the SPI0_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTD_FS_SPI0_CLK_AND_SEL_MASK 0x00010000
#define IAXXX_IO_CTRL_PORTD_FS_SPI0_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTD_FS_SPI0_CLK_AND_SEL_POS 16
#define IAXXX_IO_CTRL_PORTD_FS_SPI0_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTD_FS_SPI0_CLK_AND_SEL_DECL 16

/*
* This field is used to control FI_21_AND_SEL for pad PORTD_FS, a separate
* receiver enable for the FI_21 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTD_FS_FI_21_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_PORTD_FS_FI_21_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTD_FS_FI_21_AND_SEL_POS 22
#define IAXXX_IO_CTRL_PORTD_FS_FI_21_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTD_FS_FI_21_AND_SEL_DECL 22

/*
* This field is used to control GPIO_29_AND_SEL for pad PORTD_FS, a separate
* receiver enable for the GPIO_29 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTD_FS_GPIO_29_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_PORTD_FS_GPIO_29_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_PORTD_FS_GPIO_29_AND_SEL_POS 23
#define IAXXX_IO_CTRL_PORTD_FS_GPIO_29_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTD_FS_GPIO_29_AND_SEL_DECL 23

/*
* Function io2stmr_5_13_21_29 can be selected through the following pads:
*   COMMB_1
*   PORTB_FS
*   PORTD_FS
*   CDC_I2C_SCL
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_5_13_21_29_AND_SEL for pad PORTD_FS,
* a separate receiver enable for the IO2STMR_5_13_21_29 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTD_FS_IO2STMR_5_13_21_29_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_PORTD_FS_IO2STMR_5_13_21_29_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTD_FS_IO2STMR_5_13_21_29_AND_SEL_POS 24
#define IAXXX_IO_CTRL_PORTD_FS_IO2STMR_5_13_21_29_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTD_FS_IO2STMR_5_13_21_29_AND_SEL_DECL 24

/*** IO_CTRL_PORTE_CLK (0x500220e4) ***/
/* This register is used for pad PORTE_CLK's IO function control.Refer */
/* <a href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_PORTE_CLK_ADDR (0x500220e4)
#define IAXXX_IO_CTRL_PORTE_CLK_MASK_VAL 0x01d1c11f
#define IAXXX_IO_CTRL_PORTE_CLK_RMASK_VAL 0x01d1c11f
#define IAXXX_IO_CTRL_PORTE_CLK_WMASK_VAL 0x01d1c11f
#define IAXXX_IO_CTRL_PORTE_CLK_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad PORTE_CLK.
*   0x5: i2s_pcm4_bclk
*   0x6: i2s_cdc_clk
*   0x9: i2c1_clk
*   0xc: spi0_cs1_n
*   0xd: spi1_cs1_n
*   0xe: spi2_cs1_n
*   0x10: fo_12
*   0x12: gpio_50
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_PORTE_CLK_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_PORTE_CLK_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_PORTE_CLK_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_PORTE_CLK_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_PORTE_CLK_MUX_SEL_DECL (4:0)

/*
* This field is used to control PCM4_BCLK_AND_SEL for pad PORTE_CLK, a
* separate receiver enable for the PCM4_BCLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTE_CLK_PCM4_BCLK_AND_SEL_MASK 0x00000100
#define IAXXX_IO_CTRL_PORTE_CLK_PCM4_BCLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTE_CLK_PCM4_BCLK_AND_SEL_POS 8
#define IAXXX_IO_CTRL_PORTE_CLK_PCM4_BCLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTE_CLK_PCM4_BCLK_AND_SEL_DECL 8

/*
* Function i2c1_clk can be selected through the following pads:
*   COMMB_0
*   COMME_0
*   PORTE_CLK
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C1_CLK_AND_SEL for pad PORTE_CLK, a
* separate receiver enable for the I2C1_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTE_CLK_I2C1_CLK_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_PORTE_CLK_I2C1_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTE_CLK_I2C1_CLK_AND_SEL_POS 14
#define IAXXX_IO_CTRL_PORTE_CLK_I2C1_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTE_CLK_I2C1_CLK_AND_SEL_DECL 14

/*
* Function uart1_sin can be selected through the following pads:
*   COMMB_2
*   COMMD_2
*   PORTD_CLK
*   PORTE_CLK
*   CDC_INT_N
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control UART1_SIN_AND_SEL for pad PORTE_CLK, a
* separate receiver enable for the UART1_SIN pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTE_CLK_UART1_SIN_AND_SEL_MASK 0x00008000
#define IAXXX_IO_CTRL_PORTE_CLK_UART1_SIN_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTE_CLK_UART1_SIN_AND_SEL_POS 15
#define IAXXX_IO_CTRL_PORTE_CLK_UART1_SIN_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTE_CLK_UART1_SIN_AND_SEL_DECL 15

/*
* Function spi1_rxd can be selected through the following pads:
*   COMMB_0
*   PORTE_CLK
*   CDC_PWRON
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SPI1_RXD_AND_SEL for pad PORTE_CLK, a
* separate receiver enable for the SPI1_RXD pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTE_CLK_SPI1_RXD_AND_SEL_MASK 0x00010000
#define IAXXX_IO_CTRL_PORTE_CLK_SPI1_RXD_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTE_CLK_SPI1_RXD_AND_SEL_POS 16
#define IAXXX_IO_CTRL_PORTE_CLK_SPI1_RXD_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTE_CLK_SPI1_RXD_AND_SEL_DECL 16

/*
* This field is used to control PORTE_CLK2CNR_AND_SEL for pad PORTE_CLK, a
* separate receiver enable for the PORTE_CLK2CNR pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTE_CLK_PORTE_CLK2CNR_AND_SEL_MASK 0x00100000
#define IAXXX_IO_CTRL_PORTE_CLK_PORTE_CLK2CNR_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTE_CLK_PORTE_CLK2CNR_AND_SEL_POS 20
#define IAXXX_IO_CTRL_PORTE_CLK_PORTE_CLK2CNR_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTE_CLK_PORTE_CLK2CNR_AND_SEL_DECL 20

/*
* Function fi_24 can be selected through the following pads:
*   COMMC_0
*   PORTE_CLK
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control FI_24_AND_SEL for pad PORTE_CLK, a separate
* receiver enable for the FI_24 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTE_CLK_FI_24_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_PORTE_CLK_FI_24_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTE_CLK_FI_24_AND_SEL_POS 22
#define IAXXX_IO_CTRL_PORTE_CLK_FI_24_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTE_CLK_FI_24_AND_SEL_DECL 22

/*
* This field is used to control GPIO_50_AND_SEL for pad PORTE_CLK, a
* separate receiver enable for the GPIO_50 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTE_CLK_GPIO_50_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_PORTE_CLK_GPIO_50_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_PORTE_CLK_GPIO_50_AND_SEL_POS 23
#define IAXXX_IO_CTRL_PORTE_CLK_GPIO_50_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTE_CLK_GPIO_50_AND_SEL_DECL 23

/*
* Function io2stmr_0_8_16_24 can be selected through the following pads:
*   COMMA_2
*   PORTA_CLK
*   PORTC_CLK
*   PORTE_CLK
*   CDC_PWRON
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_0_8_16_24_AND_SEL for pad PORTE_CLK,
* a separate receiver enable for the IO2STMR_0_8_16_24 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTE_CLK_IO2STMR_0_8_16_24_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_PORTE_CLK_IO2STMR_0_8_16_24_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTE_CLK_IO2STMR_0_8_16_24_AND_SEL_POS 24
#define IAXXX_IO_CTRL_PORTE_CLK_IO2STMR_0_8_16_24_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTE_CLK_IO2STMR_0_8_16_24_AND_SEL_DECL 24

/*** IO_CTRL_PORTE_DI (0x500220e8) ***/
/* This register is used for pad PORTE_DI's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_PORTE_DI_ADDR (0x500220e8)
#define IAXXX_IO_CTRL_PORTE_DI_MASK_VAL 0x01c1c11f
#define IAXXX_IO_CTRL_PORTE_DI_RMASK_VAL 0x01c1c11f
#define IAXXX_IO_CTRL_PORTE_DI_WMASK_VAL 0x01c1c11f
#define IAXXX_IO_CTRL_PORTE_DI_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad PORTE_DI.
*   0x9: i2c2_clk
*   0xb: spi1_cs0_n
*   0xc: spi0_cs1_n
*   0xd: spi1_cs1_n
*   0xe: spi2_cs1_n
*   0x12: gpio_52
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_PORTE_DI_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_PORTE_DI_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_PORTE_DI_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_PORTE_DI_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_PORTE_DI_MUX_SEL_DECL (4:0)

/*
* This field is used to control PCM4_DR_AND_SEL for pad PORTE_DI, a separate
* receiver enable for the PCM4_DR pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTE_DI_PCM4_DR_AND_SEL_MASK 0x00000100
#define IAXXX_IO_CTRL_PORTE_DI_PCM4_DR_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTE_DI_PCM4_DR_AND_SEL_POS 8
#define IAXXX_IO_CTRL_PORTE_DI_PCM4_DR_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTE_DI_PCM4_DR_AND_SEL_DECL 8

/*
* Function i2c2_clk can be selected through the following pads:
*   COMMB_2
*   COMME_2
*   PORTE_DI
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C2_CLK_AND_SEL for pad PORTE_DI, a
* separate receiver enable for the I2C2_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTE_DI_I2C2_CLK_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_PORTE_DI_I2C2_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTE_DI_I2C2_CLK_AND_SEL_POS 14
#define IAXXX_IO_CTRL_PORTE_DI_I2C2_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTE_DI_I2C2_CLK_AND_SEL_DECL 14

/*
* Function uart0_sin can be selected through the following pads:
*   COMMA_2
*   COMMC_2
*   COMMD_0
*   PORTD_DI
*   PORTE_DI
*   CDC_PWRON
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control UART0_SIN_AND_SEL for pad PORTE_DI, a
* separate receiver enable for the UART0_SIN pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTE_DI_UART0_SIN_AND_SEL_MASK 0x00008000
#define IAXXX_IO_CTRL_PORTE_DI_UART0_SIN_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTE_DI_UART0_SIN_AND_SEL_POS 15
#define IAXXX_IO_CTRL_PORTE_DI_UART0_SIN_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTE_DI_UART0_SIN_AND_SEL_DECL 15

/*
* Function spi1_cs0_n can be selected through the following pads:
*   COMMB_2
*   PORTE_DI
*   CDC_INT_N
*   CDC_I2C_SCL
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SPI1_CS0_N_AND_SEL for pad PORTE_DI, a
* separate receiver enable for the SPI1_CS0_N pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTE_DI_SPI1_CS0_N_AND_SEL_MASK 0x00010000
#define IAXXX_IO_CTRL_PORTE_DI_SPI1_CS0_N_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTE_DI_SPI1_CS0_N_AND_SEL_POS 16
#define IAXXX_IO_CTRL_PORTE_DI_SPI1_CS0_N_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTE_DI_SPI1_CS0_N_AND_SEL_DECL 16

/*
* Function fi_26 can be selected through the following pads:
*   COMMC_2
*   PORTE_DI
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control FI_26_AND_SEL for pad PORTE_DI, a separate
* receiver enable for the FI_26 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTE_DI_FI_26_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_PORTE_DI_FI_26_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTE_DI_FI_26_AND_SEL_POS 22
#define IAXXX_IO_CTRL_PORTE_DI_FI_26_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTE_DI_FI_26_AND_SEL_DECL 22

/*
* This field is used to control GPIO_52_AND_SEL for pad PORTE_DI, a separate
* receiver enable for the GPIO_52 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTE_DI_GPIO_52_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_PORTE_DI_GPIO_52_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_PORTE_DI_GPIO_52_AND_SEL_POS 23
#define IAXXX_IO_CTRL_PORTE_DI_GPIO_52_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTE_DI_GPIO_52_AND_SEL_DECL 23

/*
* Function io2stmr_2_10_18_26 can be selected through the following pads:
*   COMMA_0
*   INTR_EVNT
*   PORTA_DI
*   PORTC_DI
*   PORTE_DI
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_2_10_18_26_AND_SEL for pad PORTE_DI,
* a separate receiver enable for the IO2STMR_2_10_18_26 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTE_DI_IO2STMR_2_10_18_26_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_PORTE_DI_IO2STMR_2_10_18_26_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTE_DI_IO2STMR_2_10_18_26_AND_SEL_POS 24
#define IAXXX_IO_CTRL_PORTE_DI_IO2STMR_2_10_18_26_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTE_DI_IO2STMR_2_10_18_26_AND_SEL_DECL 24

/*** IO_CTRL_PORTE_DO (0x500220ec) ***/
/* This register is used for pad PORTE_DO's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_PORTE_DO_ADDR (0x500220ec)
#define IAXXX_IO_CTRL_PORTE_DO_MASK_VAL 0x01c0401f
#define IAXXX_IO_CTRL_PORTE_DO_RMASK_VAL 0x01c0401f
#define IAXXX_IO_CTRL_PORTE_DO_WMASK_VAL 0x01c0401f
#define IAXXX_IO_CTRL_PORTE_DO_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad PORTE_DO.
*   0x3: pcm4_dt
*   0x9: i2c2_data
*   0xa: uart0_sout
*   0xb: spi1_txd
*   0xc: spi0_cs3_n
*   0xd: spi1_cs3_n
*   0xe: spi2_cs3_n
*   0x12: gpio_53
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_PORTE_DO_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_PORTE_DO_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_PORTE_DO_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_PORTE_DO_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_PORTE_DO_MUX_SEL_DECL (4:0)

/*
* Function i2c2_data can be selected through the following pads:
*   COMMB_3
*   COMME_3
*   PORTE_DO
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C2_DATA_AND_SEL for pad PORTE_DO, a
* separate receiver enable for the I2C2_DATA pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTE_DO_I2C2_DATA_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_PORTE_DO_I2C2_DATA_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTE_DO_I2C2_DATA_AND_SEL_POS 14
#define IAXXX_IO_CTRL_PORTE_DO_I2C2_DATA_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTE_DO_I2C2_DATA_AND_SEL_DECL 14

/*
* Function fi_27 can be selected through the following pads:
*   COMMC_3
*   PORTE_DO
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control FI_27_AND_SEL for pad PORTE_DO, a separate
* receiver enable for the FI_27 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTE_DO_FI_27_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_PORTE_DO_FI_27_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTE_DO_FI_27_AND_SEL_POS 22
#define IAXXX_IO_CTRL_PORTE_DO_FI_27_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTE_DO_FI_27_AND_SEL_DECL 22

/*
* This field is used to control GPIO_53_AND_SEL for pad PORTE_DO, a separate
* receiver enable for the GPIO_53 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTE_DO_GPIO_53_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_PORTE_DO_GPIO_53_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_PORTE_DO_GPIO_53_AND_SEL_POS 23
#define IAXXX_IO_CTRL_PORTE_DO_GPIO_53_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTE_DO_GPIO_53_AND_SEL_DECL 23

/*
* Function io2stmr_3_11_19_27 can be selected through the following pads:
*   COMMA_1
*   PORTA_DO
*   PORTC_DO
*   PORTE_DO
*   CDC_RESET_N
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_3_11_19_27_AND_SEL for pad PORTE_DO,
* a separate receiver enable for the IO2STMR_3_11_19_27 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTE_DO_IO2STMR_3_11_19_27_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_PORTE_DO_IO2STMR_3_11_19_27_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTE_DO_IO2STMR_3_11_19_27_AND_SEL_POS 24
#define IAXXX_IO_CTRL_PORTE_DO_IO2STMR_3_11_19_27_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTE_DO_IO2STMR_3_11_19_27_AND_SEL_DECL 24

/*** IO_CTRL_PORTE_FS (0x500220f0) ***/
/* This register is used for pad PORTE_FS's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_PORTE_FS_ADDR (0x500220f0)
#define IAXXX_IO_CTRL_PORTE_FS_MASK_VAL 0x01c1411f
#define IAXXX_IO_CTRL_PORTE_FS_RMASK_VAL 0x01c1411f
#define IAXXX_IO_CTRL_PORTE_FS_WMASK_VAL 0x01c1411f
#define IAXXX_IO_CTRL_PORTE_FS_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad PORTE_FS.
*   0x5: i2s_pcm4_fs
*   0x9: i2c1_data
*   0xa: uart1_sout
*   0xb: spi1_clk
*   0xc: spi0_cs2_n
*   0xd: spi1_cs2_n
*   0xe: spi2_cs2_n
*   0x10: fo_13
*   0x12: gpio_51
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_PORTE_FS_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_PORTE_FS_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_PORTE_FS_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_PORTE_FS_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_PORTE_FS_MUX_SEL_DECL (4:0)

/*
* This field is used to control PCM4_FS_AND_SEL for pad PORTE_FS, a separate
* receiver enable for the PCM4_FS pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTE_FS_PCM4_FS_AND_SEL_MASK 0x00000100
#define IAXXX_IO_CTRL_PORTE_FS_PCM4_FS_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTE_FS_PCM4_FS_AND_SEL_POS 8
#define IAXXX_IO_CTRL_PORTE_FS_PCM4_FS_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTE_FS_PCM4_FS_AND_SEL_DECL 8

/*
* Function i2c1_data can be selected through the following pads:
*   COMMB_1
*   COMME_1
*   PORTE_FS
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control I2C1_DATA_AND_SEL for pad PORTE_FS, a
* separate receiver enable for the I2C1_DATA pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTE_FS_I2C1_DATA_AND_SEL_MASK 0x00004000
#define IAXXX_IO_CTRL_PORTE_FS_I2C1_DATA_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTE_FS_I2C1_DATA_AND_SEL_POS 14
#define IAXXX_IO_CTRL_PORTE_FS_I2C1_DATA_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTE_FS_I2C1_DATA_AND_SEL_DECL 14

/*
* Function spi1_clk can be selected through the following pads:
*   COMMB_1
*   PORTE_FS
*   CDC_I2C_SDA_RD
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control SPI1_CLK_AND_SEL for pad PORTE_FS, a
* separate receiver enable for the SPI1_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTE_FS_SPI1_CLK_AND_SEL_MASK 0x00010000
#define IAXXX_IO_CTRL_PORTE_FS_SPI1_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTE_FS_SPI1_CLK_AND_SEL_POS 16
#define IAXXX_IO_CTRL_PORTE_FS_SPI1_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTE_FS_SPI1_CLK_AND_SEL_DECL 16

/*
* Function fi_25 can be selected through the following pads:
*   COMMC_1
*   PORTE_FS
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control FI_25_AND_SEL for pad PORTE_FS, a separate
* receiver enable for the FI_25 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTE_FS_FI_25_AND_SEL_MASK 0x00400000
#define IAXXX_IO_CTRL_PORTE_FS_FI_25_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTE_FS_FI_25_AND_SEL_POS 22
#define IAXXX_IO_CTRL_PORTE_FS_FI_25_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTE_FS_FI_25_AND_SEL_DECL 22

/*
* This field is used to control GPIO_51_AND_SEL for pad PORTE_FS, a separate
* receiver enable for the GPIO_51 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTE_FS_GPIO_51_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_PORTE_FS_GPIO_51_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_PORTE_FS_GPIO_51_AND_SEL_POS 23
#define IAXXX_IO_CTRL_PORTE_FS_GPIO_51_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTE_FS_GPIO_51_AND_SEL_DECL 23

/*
* Function io2stmr_1_9_17_25 can be selected through the following pads:
*   COMMA_3
*   INTR_API
*   PORTA_FS
*   PORTC_FS
*   PORTE_FS
* Please make sure you set the AND_SEL bit for only one of these pads to
* turn on this function.
*
 This field is used to control IO2STMR_1_9_17_25_AND_SEL for pad PORTE_FS,
* a separate receiver enable for the IO2STMR_1_9_17_25 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_PORTE_FS_IO2STMR_1_9_17_25_AND_SEL_MASK 0x01000000
#define IAXXX_IO_CTRL_PORTE_FS_IO2STMR_1_9_17_25_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_PORTE_FS_IO2STMR_1_9_17_25_AND_SEL_POS 24
#define IAXXX_IO_CTRL_PORTE_FS_IO2STMR_1_9_17_25_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_PORTE_FS_IO2STMR_1_9_17_25_AND_SEL_DECL 24

/*** IO_CTRL_RESET_N (0x500220f4) ***/
/* This register is used for pad RESET_N's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_RESET_N_ADDR (0x500220f4)
#define IAXXX_IO_CTRL_RESET_N_MASK_VAL 0x00000000
#define IAXXX_IO_CTRL_RESET_N_RMASK_VAL 0x00000000
#define IAXXX_IO_CTRL_RESET_N_WMASK_VAL 0x00000000
#define IAXXX_IO_CTRL_RESET_N_RESET_VAL 0x00000000

/*** IO_CTRL_SLB_CLK (0x500220f8) ***/
/* This register is used for pad SLB_CLK's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_SLB_CLK_ADDR (0x500220f8)
#define IAXXX_IO_CTRL_SLB_CLK_MASK_VAL 0x0090101f
#define IAXXX_IO_CTRL_SLB_CLK_RMASK_VAL 0x0090101f
#define IAXXX_IO_CTRL_SLB_CLK_WMASK_VAL 0x0090101f
#define IAXXX_IO_CTRL_SLB_CLK_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad SLB_CLK.
*   0x12: gpio_32
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_SLB_CLK_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_SLB_CLK_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_SLB_CLK_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_SLB_CLK_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_SLB_CLK_MUX_SEL_DECL (4:0)

/*
* This field is used to control MIPI_CLK_AND_SEL for pad SLB_CLK, a separate
* receiver enable for the MIPI_CLK pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_SLB_CLK_MIPI_CLK_AND_SEL_MASK 0x00001000
#define IAXXX_IO_CTRL_SLB_CLK_MIPI_CLK_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_SLB_CLK_MIPI_CLK_AND_SEL_POS 12
#define IAXXX_IO_CTRL_SLB_CLK_MIPI_CLK_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_SLB_CLK_MIPI_CLK_AND_SEL_DECL 12

/*
* This field is used to control SLB_CLK2CNR_AND_SEL for pad SLB_CLK, a
* separate receiver enable for the SLB_CLK2CNR pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_SLB_CLK_SLB_CLK2CNR_AND_SEL_MASK 0x00100000
#define IAXXX_IO_CTRL_SLB_CLK_SLB_CLK2CNR_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_SLB_CLK_SLB_CLK2CNR_AND_SEL_POS 20
#define IAXXX_IO_CTRL_SLB_CLK_SLB_CLK2CNR_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_SLB_CLK_SLB_CLK2CNR_AND_SEL_DECL 20

/*
* This field is used to control GPIO_32_AND_SEL for pad SLB_CLK, a separate
* receiver enable for the GPIO_32 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_SLB_CLK_GPIO_32_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_SLB_CLK_GPIO_32_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_SLB_CLK_GPIO_32_AND_SEL_POS 23
#define IAXXX_IO_CTRL_SLB_CLK_GPIO_32_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_SLB_CLK_GPIO_32_AND_SEL_DECL 23

/*** IO_CTRL_SLB_DATA (0x500220fc) ***/
/* This register is used for pad SLB_DATA's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_SLB_DATA_ADDR (0x500220fc)
#define IAXXX_IO_CTRL_SLB_DATA_MASK_VAL 0x0080101f
#define IAXXX_IO_CTRL_SLB_DATA_RMASK_VAL 0x0080101f
#define IAXXX_IO_CTRL_SLB_DATA_WMASK_VAL 0x0080101f
#define IAXXX_IO_CTRL_SLB_DATA_RESET_VAL 0x0080001f

/*
* This field is used to select a function source for output data and output
* enable control for pad SLB_DATA.
*   0x7: mipi_data
*   0x12: gpio_33
*   Otherwise: De-assert output enable. Field DDST of this register
* determines the value of the output.
*/
#define IAXXX_IO_CTRL_SLB_DATA_MUX_SEL_MASK 0x0000001f
#define IAXXX_IO_CTRL_SLB_DATA_MUX_SEL_RESET_VAL 0x1f
#define IAXXX_IO_CTRL_SLB_DATA_MUX_SEL_POS 0
#define IAXXX_IO_CTRL_SLB_DATA_MUX_SEL_SIZE 5
#define IAXXX_IO_CTRL_SLB_DATA_MUX_SEL_DECL (4:0)

/*
* This field is used to control MIPI_DATA_AND_SEL for pad SLB_DATA, a
* separate receiver enable for the MIPI_DATA pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_SLB_DATA_MIPI_DATA_AND_SEL_MASK 0x00001000
#define IAXXX_IO_CTRL_SLB_DATA_MIPI_DATA_AND_SEL_RESET_VAL 0x0
#define IAXXX_IO_CTRL_SLB_DATA_MIPI_DATA_AND_SEL_POS 12
#define IAXXX_IO_CTRL_SLB_DATA_MIPI_DATA_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_SLB_DATA_MIPI_DATA_AND_SEL_DECL 12

/*
* This field is used to control GPIO_33_AND_SEL for pad SLB_DATA, a separate
* receiver enable for the GPIO_33 pad function.
*   0x0: disable receive function, input to core logic driven to "0".
*   0x1: enable receive function.
*/
#define IAXXX_IO_CTRL_SLB_DATA_GPIO_33_AND_SEL_MASK 0x00800000
#define IAXXX_IO_CTRL_SLB_DATA_GPIO_33_AND_SEL_RESET_VAL 0x1
#define IAXXX_IO_CTRL_SLB_DATA_GPIO_33_AND_SEL_POS 23
#define IAXXX_IO_CTRL_SLB_DATA_GPIO_33_AND_SEL_SIZE 1
#define IAXXX_IO_CTRL_SLB_DATA_GPIO_33_AND_SEL_DECL 23

/*** IO_CTRL_TEST (0x50022100) ***/
/* This register is used for pad TEST's IO function control.Refer <a */
/* href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_TEST_ADDR (0x50022100)
#define IAXXX_IO_CTRL_TEST_MASK_VAL 0x00000000
#define IAXXX_IO_CTRL_TEST_RMASK_VAL 0x00000000
#define IAXXX_IO_CTRL_TEST_WMASK_VAL 0x00000000
#define IAXXX_IO_CTRL_TEST_RESET_VAL 0x00000000

/*** IO_CTRL_TRACE_CLK (0x50022104) ***/
/* This register is used for pad TRACE_CLK's IO function control.Refer */
/* <a href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_TRACE_CLK_ADDR (0x50022104)
#define IAXXX_IO_CTRL_TRACE_CLK_MASK_VAL 0x00000000
#define IAXXX_IO_CTRL_TRACE_CLK_RMASK_VAL 0x00000000
#define IAXXX_IO_CTRL_TRACE_CLK_WMASK_VAL 0x00000000
#define IAXXX_IO_CTRL_TRACE_CLK_RESET_VAL 0x00000000

/*** IO_CTRL_TRACE_DATA (0x50022108) ***/
/* This register is used for pad TRACE_DATA's IO function control.Refer */
/* <a href="#io_pad_control_diagram">IO Function and PAD Control</a> to */
/* understand these controls better. Refer to <a */
/* spreadsheet</a>'s 'Func' tab to get all functions supported on this pin. */
#define IAXXX_IO_CTRL_TRACE_DATA_ADDR (0x50022108)
#define IAXXX_IO_CTRL_TRACE_DATA_MASK_VAL 0x00000000
#define IAXXX_IO_CTRL_TRACE_DATA_RMASK_VAL 0x00000000
#define IAXXX_IO_CTRL_TRACE_DATA_WMASK_VAL 0x00000000
#define IAXXX_IO_CTRL_TRACE_DATA_RESET_VAL 0x00000000

#endif /* __IAXXX_REGISTER_DEFS_IOCTRL_H__ */
