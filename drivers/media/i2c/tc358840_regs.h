/*
 * tc358840.h - Toshiba HDMI-CSI bridge registers
 *
 * Copyright (c) 2015, Armin Weiss <weii@zhaw.ch>
 * Copyright (c) 2017, Plamen Valev <pvalev@mm-sol.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __TC358840_REGS_H_
#define __TC358840_REGS_H_

/* *** General (16 bit) *** */
#define TC358840_CHIPID_ADDR			0x0000
#define MASK_CHIPID				0xFF00
#define MASK_REVID				0x00FF
#define TC358840_CHIPID				0x4700

#define TC358840_SYSCTL				0x0002
#define MASK_SLEEP				(1 << 0)
#define MASK_I2SDIS				(1 << 7)
#define MASK_HDMIRST				(1 << 8)
#define MASK_CTXRST				(1 << 9)
#define MASK_CECRST				(1 << 10)
#define MASK_IRRST				(1 << 11)
#define MASK_SPLRST				(1 << 12)
#define MASK_ABRST				(1 << 14)
#define MASK_RESET_ALL				0x5F80

#define TC358840_CONFCTL0			0x0004
#define MASK_VTX0EN				(1 << 0)
#define MASK_VTX1EN				(1 << 1)
#define MASK_AUTOINDEX				(1 << 2)
#define MASK_AUDOUTSEL_CSITX0			(0 << 3)
#define MASK_AUDOUTSEL_CSITX1			(1 << 3)
#define MASK_AUDOUTSEL_I2S			(2 << 3)
#define MASK_AUDOUTSEL_TDM			(3 << 3)
#define MASK_ABUFEN				(1 << 5)
#define MASK_YCBCRFMT				(3 << 6)
#define MASK_YCBCRFMT_YCBCR444			(0 << 6)
#define MASK_YCBCRFMT_YCBCR422_12		(1 << 6)
#define MASK_YCBCRFMT_VPID2			(2 << 6)
#define MASK_YCBCRFMT_YCBCR422_8		(3 << 6)
#define MASK_I2SDLYOPT				(1 << 8)
#define MASK_AUDCHSEL				(1 << 9)
#define MASK_AUDCHNUM_8				(0 << 10)
#define MASK_AUDCHNUM_6				(1 << 10)
#define MASK_AUDCHNUM_4				(2 << 10)
#define MASK_AUDCHNUM_2				(3 << 10)
#define MASK_ACLKOPT				(1 << 12)
#define MASK_IECEN				(1 << 13)
#define MASK_SLMBEN				(1 << 14)
#define MASK_TX_MSEL				(1 << 15)

#define TC358840_CONFCTL1			0x0006
#define MASK_TX_OUT_FMT				0x0003
#define MASK_TX_OUT_FMT_RGB888			(0 << 0)
#define MASK_TX_OUT_FMT_RGB666			(2 << 0)
#define MASK_TX_MS_EN				(1 << 2)

#define TC358840_I2SIOCTL			0x0072
#define MASK_I2SCTL_1				(1 << 0)
#define MASK_I2SCTL_2				(1 << 1)
#define MASK_I2SCTL_3				(1 << 2)
#define MASK_I2SCTL_4				(1 << 3)

/* *** Interrupt (16 bit) *** */
#define TC358840_INTSTATUS			0x0014
#define TC358840_INTMASK			0x0016
#define MASK_IR_DINT				(1 << 0)
#define MASK_IR_EINT				(1 << 1)
#define MASK_CEC_RINT				(1 << 2)
#define MASK_CEC_TINT				(1 << 3)
#define MASK_CEC_EINT				(1 << 4)
#define MASK_SYS_INT				(1 << 5)
#define MASK_CSITX0_INT				(1 << 8)
#define MASK_HDMI_INT				(1 << 9)
#define MASK_AMUTE_INT				(1 << 10)
#define MASK_CSITX1_INT				(1 << 11)
#define MASK_INT_STATUS_MASK_ALL		0x0F3F

/* *** Interrupt and MASKs (8 bit) *** */
#define TC358840_HDMI_INT0			0x8500
#define MASK_MISC				0x02
#define MASK_KEY				0x80

#define TC358840_HDMI_INT1			0x8501
#define MASK_SYS				0x01
#define MASK_CLK				0x02
#define MASK_PACKET				0x04
#define MASK_ACBIT				0x08
#define MASK_AUD				0x10
#define MASK_ERR				0x20
#define MASK_HDCP				0x40
#define MASK_GBD				0x80

#define TC358840_SYS_INT			0x8502
#define TC358840_SYS_INTM			0x8512
#define MASK_DDC				0x01
#define MASK_TMDS				0x02
#define MASK_DPMBDET				0x04
#define MASK_NOPMBDET				0x08
#define MASK_HDMI				0x10
#define MASK_DVI				0x20
#define MASK_ACRN				0x40
#define MASK_ACR_CTS				0x80

#define TC358840_CLK_INT			0x8503
#define TC358840_CLK_INTM			0x8513
#define MASK_TMDSCLK_CHG			0x01
#define MASK_PHYCLK_CHG				0x02
#define MASK_PXCLK_CHG				0x04
#define MASK_DC_CHG				0x08
#define MASK_IN_HV_CHG				0x10
#define MASK_IN_DE_CHG				0x20
#define MASK_OUT_H_CHG				0x40
#define MASK_OUT_DE_CHG				0x80

#define TC358840_PACKET_INT			0x8504
#define TC358840_PACKET_INTM			0x8514
#define MASK_PK_AVI				0x01
#define MASK_PK_AUD				0x02
#define MASK_PK_MS				0x04
#define MASK_PK_SPD				0x08
#define MASK_PK_VS				0x10
#define MASK_PK_ACP				0x20
#define MASK_PK_ISRC				0x40
#define MASK_PK_ISRC2				0x80

#define TC358840_CBIT_INT			0x8505
#define TC358840_CBIT_INTM			0x8515
#define MASK_CBIT				0x01
#define MASK_CBIT_FS				0x02
#define MASK_CBIT_NLPCM				0x04
#define MASK_AU_HBR				0x08
#define MASK_AU_DSD				0x10
#define MASK_AF_UNLOCK				0x40
#define MASK_AF_LOCK				0x80

#define TC358840_AUDIO_INT			0x8506
#define TC358840_AUDIO_INTM			0x8516
#define MASK_BUFINIT_END			0x01
#define MASK_BUF_UNDER				0x02
#define MASK_BUF_NU2				0x04
#define MASK_BUF_NU1				0x08
#define MASK_BUF_CENTER				0x10
#define MASK_BUF_NO1				0x20
#define MASK_BUF_NO2				0x40
#define MASK_BUF_OVER				0x80

#define TC358840_ERR_INT			0x8507
#define TC358840_ERR_INTM			0x8517
#define MASK_DC_PPERR				0x01
#define MASK_DC_BUFERR				0x02
#define MASK_DC_DEERR				0x04
#define MASK_DC_NOCD				0x08
#define MASK_NO_AVI				0x10
#define MASK_NO_ACP				0x20
#define MASK_AU_FRAME				0x40
#define MASK_EESS_ERR				0x80

#define TC358840_HDCP_INT			0x8508
#define TC358840_HDCP_INTM			0x8518
#define MASK_AN_END				0x01
#define MASK_AKSV_END				0x02
#define MASK_KM_END				0x04
#define MASK_R0_END				0x08
#define MASK_SHA_END				0x10
#define MASK_LINKERR				0x20
#define MASK_AVM_CLR				0x40
#define MASK_AVM_SET				0x80

#define TC358840_GBD_INT			0x8509
#define TC358840_GBD_INTM			0x8519
#define MASK_GBD_ON				0x01
#define MASK_GBD_OFF				0x02
#define MASK_P1GBD_DET				0x04
#define MASK_P0GBD_CHG				0x10
#define MASK_P1GBD_CHG				0x20
#define MASK_GBD_ACLR				0x40
#define MASK_GBD_PKERR				0x80

#define TC358840_MISC_INT			0x850B
#define TC358840_MISC_INTM			0x851B
#define MASK_AUDIO_MUTE				0x01
#define MASK_SYNC_CHG				0x02
#define MASK_NO_VS				0x04
#define MASK_NO_SPD				0x08
#define MASK_AS_LAYOUT				0x10
#define MASK_VIDEO_COLOR			0x20
#define MASK_AU_HBR_OFF				0x40
#define MASK_AU_DSD_OFF				0x80

/* *** STATUS *** */
#define TC358840_SYS_STATUS			0x8520
#define MASK_S_SYNC				0x80
#define MASK_S_AVMUTE				0x40
#define MASK_S_HDCP				0x20
#define MASK_S_HDMI				0x10
#define MASK_S_PHY_SCDT				0x08
#define MASK_S_PHY_PLL				0x04
#define MASK_S_TMDS				0x02
#define MASK_S_DDC5V				0x01

#define TC358840_VI_STATUS1			0x8522
#define MASK_S_V_GBD				0x08
#define MASK_S_DEEPCOLOR			0x0c
#define MASK_S_V_422				0x02
#define MASK_S_V_INTERLACE			0x01

#define TC358840_VI_STATUS3			0x8528
#define MASK_S_V_COLOR				0x1F
#define MASK_RGB_FULL				0x00
#define MASK_RGB_LIMITED			0x01
#define MASK_YCBCR601_FULL			0x02
#define MASK_YCBCR601_LIMITED			0x03
#define MASK_ADOBE_RGB_FULL			0x04
#define MASK_ADOBE_RGB_LIMITED			0x05
#define MASK_YCBCR709_FULL			0x06
#define MASK_YCBCR709_LIMITED			0x07
#define MASK_XVYCC601_FULL			0x0A
#define MASK_XVYCC601_LIMITED			0x0B
#define MASK_XVYCC709_FULL			0x0E
#define MASK_XVYCC709_LIMITED			0x0F
#define MASK_SYCC601_FULL			0x12
#define MASK_SYCC601_LIMITED			0x13
#define MASK_ADOBE_YCC601_FULL			0x1A
#define MASK_ADOBE_YCC601_LIMITED		0x1B


/* *** CSI TX (32 bit) *** */
#define CSITX0_BASE_ADDR			0x0000
#define CSITX1_BASE_ADDR			0x0200

#define TC358840_CSITX_CLKEN			0x0108
#define MASK_CSITX_EN				(1 << 0)

#define TC358840_PPICLKEN			0x010C
#define MASK_HSTXCLKEN				0x00000001

#define TC358840_MODECONF			0x0110	/* Not in Ref. v1.5 */
#define TC358840_MODECONF2			0x0310	/* Not in Ref. v1.5 */
#define MASK_CSI2MODE				(1 << 0)
#define MASK_VSYNC_POL_SW			(1 << 1)
#define MASK_HSYNC_POL_SW			(1 << 2)
#define MASK_DTVALID_POL_SW			(1 << 3)
#define MASK_INDMODE				(1 << 4)

#define TC358840_LANEEN				0x0118
#define MASK_LANES				0x00000007
#define MASK_LANE_0_EN				(1 << 0)
#define MASK_LANE_0_1_EN			(2 << 0)
#define MASK_LANE_0_1_2_EN			(3 << 0)
#define MASK_LANE_0_1_2_3_EN			(4 << 0)
#define MASK_LANES				0x00000007
#define MASK_CLANEEN				(1 << 4)

#define TC358840_CSITX_START			0x011C
#define TC358840_LINEINITCNT			0x0120
#define TC358840_HSTOCNT			0x0124

#define TC358840_INTEN				0x0128	/* Not in Ref. v1.5 */
#define MASK_VH_DLY_EN				(1 << 0)
#define MASK_VFHSYNCMASK_EN			(1 << 7)
#define MASK_IND_MODE_SEL_PORT			(0 << 8)
#define MASK_IND_MODE_SEL_REG			(1 << 8)
#define MASK_IND_TO_EN				(1 << 9)
#define MASK_HSTX_TO_EN				(1 << 10)
#define MASK_LRX_H_TO_EN			(1 << 11)
#define MASK_TA_TO_EN				(1 << 12)
#define MASK_PR_TO_EN				(1 << 13)
#define MASK_PRESP_TO_EN			(1 << 14)
#define MASK_DSI_RX_STATE_INT_EN		(1 << 16)
#define MASK_DSI_RX_TRIG_INT_EN			(1 << 17)
#define MASK_DSI_LP_TX_INT_EN			(1 << 18)
#define MASK_DSI_RX_ERR_INT_EN			(1 << 19)
#define MASK_DSI_RP_TO_INT_EN			(1 << 20)
#define MASK_APP_SIDE_ERR_INT_EN		(1 << 21)
#define MASK_INIT_INT_EN			(1 << 22)
#define MASK_DEBUG_MODE_EN			(1 << 31)

#define TC358840_BTATOCNT			0x0130	/* Not in Ref. v1.5 */
#define TC358840_PERI_BTACNT			0x0134	/* Not in Ref. v1.5 */
#define TC358840_LP_READCNT			0x0138	/* Not in Ref. v1.5 */
#define TC358840_LP_WRITECNT			0x013C	/* Not in Ref. v1.5 */

#define TC358840_HSREADCNT			0x0140	/* Not in Ref. v1.5 */
#define TC358840_HSWRITECNT			0x0144	/* Not in Ref. v1.5 */
#define TC358840_PERIRSTCNT			0x0148	/* Not in Ref. v1.5 */
#define TC358840_LRXHTOCNT			0x014C	/* Not in Ref. v1.5 */

#define TC358840_FUNCMODE			0x0150
#define MASK_CONTCLKMODE			(1 << 5)
#define MASK_FORCESTOP				(1 << 10)

#define TC358840_RX_VC_EN			0x0154	/* Not in Ref. v1.5 */
#define MASK_RX_VC0				(1 << 0)
#define MASK_RX_VC1				(1 << 1)
#define MASK_RX_VC2				(1 << 2)
#define MASK_RX_VC3				(1 << 3)

#define TC358840_INPUTTOCNT			0x0158	/* Not in Ref. v1.5 */
#define TC358840_INITINTSTAT			0x0160

#define TC358840_HSYNCSTOPCNT			0x0168	/* Not in Ref. v1.5 */
#define TC358840_VHDELAY			0x0170	/* Not in Ref. v1.5 */
#define TC358840_DSITXMODE			0x017C	/* Not in Ref. v1.5 */
#define TC358840_HSYNCWIDTH			0x018C	/* Not in Ref. v1.5 */
#define TC358840_HBPR				0x0190	/* Not in Ref. v1.5 */
#define TC358840_LPRX_INT_MASK			0x01A4	/* Not in Ref. v1.5 */
#define TC358840_LPRX_FIFO_INTCOUNT		0x010C	/* Not in Ref. v1.5 */

#define TC358840_APPERRMASK			0x0214	/* Not in Ref. v1.5 */
#define TC358840_LPRX_ERR_MASK			0x021C	/* Not in Ref. v1.5 */
#define TC358840_LPTX_DONE_MASK			0x0224	/* Not in Ref. v1.5 */

#define TC358840_LPTXTIMECNT			0x0254
#define TC358840_TCLK_HEADERCNT			0x0258
#define TC358840_TCLK_TRAILCNT			0x025C
#define TC358840_THS_HEADERCNT			0x0260
#define TC358840_TWAKEUP			0x0264
#define TC358840_TCLK_POSTCNT			0x0268
#define TC358840_THS_TRAILCNT			0x026C
#define TC358840_HSTXVREGCNT			0x0270
#define TC358840_HSTXVREGEN			0x0274
#define MASK_D3M_HSTXVREGEN			0x0010
#define MASK_D2M_HSTXVREGEN			0x0008
#define MASK_D1M_HSTXVREGEN			0x0004
#define MASK_D0M_HSTXVREGEN			0x0002
#define MASK_CLM_HSTXVREGEN			0x0001

#define TC358840_BTA_COUNT			0x0278	/* Not in Ref. v1.5 */
#define TC358840_DPHY_TX_ADJUST			0x027C	/* Not in Ref. v1.5 */

#define TC358840_MIPICLKEN			0x02A0
#define MASK_MP_ENABLE				0x00000001
#define MASK_MP_CKEN				0x00000002

#define TC358840_PLLLOCKCNT			0x02A4
#define TC358840_PLLLOCK			0x02A8

#define TC358840_PLLCONF			0x02AC
#define MASK_LFBREN				(1 << 9)
#define MASK_MPLBW				0x00030000
#define MASK_MPLBW_25				(0 << 16)
#define MASK_MPLBW_33				(1 << 16)
#define MASK_MPLBW_50				(2 << 16)
#define MASK_MPLBW_MAX				(3 << 16)
#define MASK_PLL_FBD				0x000000FF
#define SET_PLL_FBD(fbd)			(((fbd) - 1) & MASK_PLL_FBD)
#define MASK_PLL_FRS				0x00000C00
#define SET_PLL_FRS(frs)			(((frs) << 10) & MASK_PLL_FRS)
#define MASK_PLL_PRD				0x0000F000
#define SET_PLL_PRD(prd)			((((prd) - 1) << 12) & \
						  MASK_PLL_PRD)
#define MASK_PLL_LBW				0x00030000
#define SET_PLL_LBW(lbw)			((((lbw) - 1) << 16) & \
						  MASK_PLL_LBW)

/* *** Split Control (16 bit) *** */
#define TC358840_STX0_CTRL			0x5000
#define TC358840_STX1_CTRL			0x5080
#define MASK_LCD_CSEL				0x0001
#define MASK_IFEN				0x0002
#define MASK_SPBP				0x0100
#define MASK_SPEN				0x0000

#define TC358840_STX0_PACKETID1 0x5002
#define TC358840_SPLITTX0_WC			0x5008	/*Removed in rev. 1.1*/
#define TC358840_SPLITTX1_WC			0x5088	/*Removed in rev. 1.1*/
#define MASK_WC						0x0F00

#define TC358840_STX0_FPX			0x500C
#define TC358840_STX0_LPX			0x500E
#define TC358840_STX1_FPX			0x508C
#define MASK_EHW				0x0FFF
/* NOTE: Only available for TX0 */
#define MASK_TX1SEL				0x4000
/* NOTE: Only available for TX0 */
#define MASK_FPXV				0x8000

/* *** HDMI PHY (8 bit) *** */
#define TC358840_PHY_CTL			0x8410
/* TODO: Check name of mask */
#define MASK_POWERCTL				(1 << 0)
/* TODO: Check name of mask */
#define MASK_48_MHZ				(1 << 1)

#define TC358840_PHY_ENB			0x8413
#define MASK_ENABLE_PHY				0x01

#define TC358840_PHY_RST			0x8414
#define MASK_RESET_CTRL				0x01	/* Reset active low */

#define TC358840_PHY_EQ_CTL			0x8420

#define TC358840_APPL_CTL			0x84F0
#define MASK_APLL_ON				0x01
#define MASK_APLL_CPCTL				0x30
#define MASK_APLL_CPCTL_HIZ			0x00
#define MASK_APLL_CPCTL_LFIX			0x10
#define MASK_APLL_CPCTL_HFIX			0x20
#define MASK_APLL_CPCTL_NORMAL			0x30

#define TC358840_DDCIO_CTL			0x84F4
#define MASK_DDC_PWR_ON				(1 << 0)

/** *** HDMI Clock (8 bit) *** */
#define TC358840_AU_STATUS0			0x8523
#define MASK_S_A_SAMPLE				0x01

#define TC358840_SYS_FREQ0			0x8540
#define TC358840_SYS_FREQ1			0x8541
#define TC358840_LOCK_REF_FREQA			0x8630
#define TC358840_LOCK_REF_FREQB			0x8631
#define TC358840_LOCK_REF_FREQC			0x8632

#define TC358840_FS_SET				0x8621
#define MASK_FS					0x0F

#define TC358840_NCO_F0_MOD			0x8670
#define MASK_NCO_F0_MOD_42MHZ			0x00
#define MASK_NCO_F0_MOD_REG			0x02

#define TC358840_NCO_48F0A			0x8671
#define TC358840_NCO_48F0B			0x8672
#define TC358840_NCO_48F0C			0x8673
#define TC358840_NCO_48F0D			0x8674
#define TC358840_NCO_44F0A			0x8675
#define TC358840_NCO_44F0B			0x8676
#define TC358840_NCO_44F0C			0x8677
#define TC358840_NCO_44F0D			0x8678

#define TC358840_SCLK_CSC0			0x8A0C
#define TC358840_SCLK_CSC1			0x8A0D

/* *** VI *** */
#define TC358840_VI_MODE			0x8570
/* TODO: Probably wrong bit (see p. 292 rev. 0.93) */
#define MASK_RGB_DVI				0x08

#define TC358840_DE_HSIZE_LO			0x8582
#define TC358840_DE_HSIZE_HI			0x8583
#define TC358840_DE_VSIZE_LO			0x858C
#define TC358840_DE_VSIZE_HI			0x858D

#define TC358840_IN_HSIZE_LO			0x858E
#define TC358840_IN_HSIZE_HI			0x858F

#define TC358840_IN_VSIZE_LO			0x8590
#define TC358840_IN_VSIZE_HI			0x8591

#define TC358840_FV_CNT_LO			0x85C1	/* Not in Ref. v1.5 */
#define TC358840_FV_CNT_HI			0x85C2	/* Not in Ref. v1.5 */

/* *** EDID (8 bit) *** */
#define TC358840_EDID_MODE			0x85E0
#define MASK_DIRECT				0x00
#define MASK_RAM_DDC2B				(1 << 0)
#define MASK_RAM_EDDC				(1 << 1)
#define MASK_EDID_MODE_ALL			0x03

#define TC358840_EDID_LEN1			0x85E3
#define TC358840_EDID_LEN2			0x85E4

#define TC358840_EDID_RAM			0x8C00

/* *** HDCP *** */
#define TC358840_BKSV				0x8800

/* *** Video Output Format (8 bit) *** */
#define TC358840_VOUT_FMT			0x8A00
#define MASK_OUTFMT_444_RGB			(0 << 0)
#define MASK_OUTFMT_422				(1 << 0)
#define MASK_OUTFMT_THROUGH			(2 << 0)
#define MASK_422FMT_NORMAL			(0 << 4)
#define MASK_422FMT_HDMITHROUGH			(1 << 4)

#define TC358840_VOUT_FIL			0x8A01
#define MASK_422FIL				0x07
#define MASK_422FIL_2_TAP			(0 << 0)
#define MASK_422FIL_3_TAP			(1 << 0)
#define MASK_422FIL_NO_FILTER			(2 << 0)
#define MASK_422FIL_2_TAP_444			(3 << 0)
#define MASK_422FIL_3_TAP_444			(4 << 0)
#define MASK_422FIL_2_TAP_444_CSC		(5 << 0)
#define MASK_422FIL_3_TAP_444_CSC		(6 << 0)
#define MASK_444FIL				0x10
#define MASK_444FIL_REPEAT			(0 << 4)
#define MASK_444FIL_2_TAP			(1 << 4)

#define TC358840_VOUT_SYNC0			0x8A02
#define MASK_MODE_2				(2 << 0)
#define MASK_MODE_3				(3 << 0)
#define MASK_M3_HSIZE				0x30
#define MASK_M3_VSIZE				0xC0

#define TC358840_VOUT_CSC			0x8A08
#define MASK_CSC_MODE				0x03
#define MASK_CSC_MODE_OFF			(0 << 0)
#define MASK_CSC_MODE_BUILTIN			(1 << 0)
#define MASK_CSC_MODE_AUTO			(2 << 0)
#define MASK_CSC_MODE_HOST			(3 << 0)
#define MASK_COLOR				0x70
#define MASK_COLOR_RGB_FULL			(0 << 4)
#define MASK_COLOR_RGB_LIMITED			(1 << 4)
#define MASK_COLOR_601_YCBCR_FULL		(2 << 4)
#define MASK_COLOR_601_YCBCR_LIMITED		(3 << 4)
#define MASK_COLOR_709_YCBCR_FULL		(4 << 4)
#define MASK_COLOR_709_YCBCR_LIMITED		(5 << 4)
#define MASK_COLOR_FULL_TO_LIMITED		(6 << 4)
#define MASK_COLOR_LIMITED_TO_FULL		(7 << 4)


/* *** HDMI Audio RefClk (8 bit) *** */
#define TC358840_FORCE_MUTE			0x8600
#define MASK_FORCE_DMUTE			(1 << 0)
#define MASK_FORCE_AMUTE			(1 << 4)

#define TC358840_AUTO_CMD0			0x8602
#define MASK_AUTO_MUTE7				0x80
#define MASK_AUTO_MUTE6				0x40
#define MASK_AUTO_MUTE5				0x20
#define MASK_AUTO_MUTE4				0x10
#define MASK_AUTO_MUTE3				0x08
#define MASK_AUTO_MUTE2				0x04
#define MASK_AUTO_MUTE1				0x02
#define MASK_AUTO_MUTE0				0x01

#define TC358840_AUTO_CMD1			0x8603
#define MASK_AUTO_MUTE10			0x04
#define MASK_AUTO_MUTE9				0x02
#define MASK_AUTO_MUTE8				0x01

#define TC358840_AUTO_CMD2			0x8604
#define MASK_AUTO_PLAY3				0x08
#define MASK_AUTO_PLAY2				0x04

#define TC358840_BUFINIT_START			0x8606
#define SET_BUFINIT_START_MS(milliseconds)	((milliseconds) / 100)

#define TC358840_FS_MUTE			0x8607
#define MASK_FS_ELSE_MUTE			0x80
#define MASK_FS22_MUTE				0x40
#define MASK_FS24_MUTE				0x20
#define MASK_FS88_MUTE				0x10
#define MASK_FS96_MUTE				0x08
#define MASK_FS176_MUTE				0x04
#define MASK_FS192_MUTE				0x02
#define MASK_FS_NO_MUTE				0x01

#define TC358840_MUTE_MODE			0x8608
#define MASK_AMUTE_DLY_0			(0 << 6)
#define MASK_AMUTE_DLY_100			(1 << 6)
#define MASK_AMUTE_DLY_200			(2 << 6)
#define MASK_AMUTE_DLY_300			(3 << 6)
#define MASK_AMUTE_POL				0x10
#define MASK_O_AMUTE_EN				0x08
#define MASK_MUTE_LRCK				0x04
#define MASK_MUTE_BCK				0x02
#define MASK_MUTE_SDO				0x01

#define TC358840_FS_IMODE			0x8620
#define MASK_NLPCM_HMODE			0x40
#define MASK_NLPCM_SMODE			0x20
#define MASK_NLPCM_IMODE			0x10
#define MASK_FS_HMODE				0x08
#define MASK_FS_AMODE				0x04
#define MASK_FS_SMODE				0x02
#define MASK_FS_IMODE				0x01

#define TC358840_ACR_MODE			0x8640
#define MASK_ACR_LOAD				0x10
#define MASK_N_MODE				0x04
#define MASK_CTS_MODE				0x01

#define TC358840_ACR_MDF0			0x8641
#define MASK_ACR_L2MDF				0x70
#define MASK_ACR_L2MDF_0_PPM			0x00
#define MASK_ACR_L2MDF_61_PPM			0x10
#define MASK_ACR_L2MDF_122_PPM			0x20
#define MASK_ACR_L2MDF_244_PPM			0x30
#define MASK_ACR_L2MDF_488_PPM			0x40
#define MASK_ACR_L2MDF_976_PPM			0x50
#define MASK_ACR_L2MDF_1976_PPM			0x60
#define MASK_ACR_L2MDF_3906_PPM			0x70
#define MASK_ACR_L1MDF				0x07
#define MASK_ACR_L1MDF_0_PPM			0x00
#define MASK_ACR_L1MDF_61_PPM			0x01
#define MASK_ACR_L1MDF_122_PPM			0x02
#define MASK_ACR_L1MDF_244_PPM			0x03
#define MASK_ACR_L1MDF_488_PPM			0x04
#define MASK_ACR_L1MDF_976_PPM			0x05
#define MASK_ACR_L1MDF_1976_PPM			0x06
#define MASK_ACR_L1MDF_3906_PPM			0x07

#define TC358840_ACR_MDF1			0x8642
#define MASK_ACR_L3MDF				0x07
#define MASK_ACR_L3MDF_0_PPM			0x00
#define MASK_ACR_L3MDF_61_PPM			0x01
#define MASK_ACR_L3MDF_122_PPM			0x02
#define MASK_ACR_L3MDF_244_PPM			0x03
#define MASK_ACR_L3MDF_488_PPM			0x04
#define MASK_ACR_L3MDF_976_PPM			0x05
#define MASK_ACR_L3MDF_1976_PPM			0x06
#define MASK_ACR_L3MDF_3906_PPM			0x07

#define TC358840_SDO_MODE1			0x8652
#define MASK_SDO_16BIT_D8BIT			0x00
#define MASK_SDO_16BIT_D9BIT			0x10
#define MASK_SDO_18BIT_D6BIT			0x20
#define MASK_SDO_18BIT_D7BIT			0x30
#define MASK_SDO_20BIT_D4BIT			0x40
#define MASK_SDO_20BIT_D5BIT			0x50
#define MASK_SDO_24BIT_D0BIT			0x60
#define MASK_SDO_MUTE				0x70
#define MASK_SDO_FMT_RIGHT			0x00
#define MASK_SDO_FMT_LEFT			0x01
#define MASK_SDO_FMT_I2S			0x02

#define TC358840_SDO0_ASSIGN			0x8653
#define TC358840_SDO1_ASSIGN			0x8654
#define TC358840_SDO2_ASSIGN			0x8655
#define TC358840_SDO3_ASSIGN			0x8656
#define SDO_RLCHANNELS_MUTE			0x00
#define HDMI_TO_SDO_RLCHANNELS(R, L)	(R<<4|L)
/*According to CEA-861-D*/
#define FL	1
#define FR	2
#define LFE	3
#define FC	4
#define RL	5
#define RR	6
#define FLC	7
#define FRC	8

#define TC358840_DIV_MODE			0x8665 /* Not in REF_01 */
#define MASK_DIV_DLY				0xf0
#define SET_DIV_DLY_MS(milliseconds)		((((milliseconds)/100) << 4) & \
						MASK_DIV_DLY)
#define MASK_DIV_MODE				0x01

#define TC358840_HDMIAUDIO_MODE			0x8680

/* *** HDMI General (16 bit) *** */
#define TC358840_DDC_CTL			0x8543
#define MASK_DDC_ACTION				0x04
#define MASK_DDC5V_MODE				0x03
#define MASK_DDC5V_MODE_0MS			0x00
#define MASK_DDC5V_MODE_50MS			0x01
#define MASK_DDC5V_MODE_100MS			0x02
#define MASK_DDC5V_MODE_200MS			0x03

#define TC358840_HPD_CTL			0x8544
#define MASK_HPD_OUT0				(1 << 0)
#define MASK_HPD_CTL0				(1 << 4)

#define TC358840_INIT_END			0x854A
#define MASK_INIT_END				0x01

/* *** Video Mute *** */
#define TC358840_VI_MUTE			0x857F
#define MASK_AUTO_MUTE				0xC0
#define MASK_VI_MUTE				0x10
#define MASK_VI_BLACK				0x01

/* *** Info Frame *** */
#define TC358840_PK_INT_MODE			0x8709
#define MASK_ISRC2_INT_MODE			0x80
#define MASK_ISRC_INT_MODE			0x40
#define MASK_ACP_INT_MODE			0x20
#define MASK_VS_INT_MODE			0x10
#define MASK_SPD_INT_MODE			0x08
#define MASK_MS_INT_MODE			0x04
#define MASK_AUD_INT_MODE			0x02
#define MASK_AVI_INT_MODE			0x01

#define TC358840_NO_PKT_LIMIT			0x870B
#define MASK_NO_ACP_LIMIT			0xF0
#define SET_NO_ACP_LIMIT_MS(milliseconds)	((((milliseconds)/80) << 4) & \
						  MASK_NO_ACP_LIMIT)

#define TC358840_NO_PKT_CLR			0x870C
#define MASK_NO_VS_CLR				0x40
#define MASK_NO_SPD_CLR				0x20
#define MASK_NO_ACP_CLR				0x10
#define MASK_NO_AVI_CLR1			0x02
#define MASK_NO_AVI_CLR0			0x01

#define ERR_PK_LIMIT				0x870D
#define NO_PKT_LIMIT2				0x870E
#define PK_AVI_0HEAD				0x8710
#define PK_AVI_1HEAD				0x8711
#define PK_AVI_2HEAD				0x8712
#define PK_AVI_0BYTE				0x8713
#define PK_AVI_1BYTE				0x8714
#define PK_AVI_2BYTE				0x8715
#define PK_AVI_3BYTE				0x8716
#define PK_AVI_4BYTE				0x8717
#define PK_AVI_5BYTE				0x8718
#define PK_AVI_6BYTE				0x8719
#define PK_AVI_7BYTE				0x871A
#define PK_AVI_8BYTE				0x871B
#define PK_AVI_9BYTE				0x871C
#define PK_AVI_10BYTE				0x871D
#define PK_AVI_11BYTE				0x871E
#define PK_AVI_12BYTE				0x871F
#define PK_AVI_13BYTE				0x8720
#define PK_AVI_14BYTE				0x8721
#define PK_AVI_15BYTE				0x8722
#define PK_AVI_16BYTE				0x8723

#define NO_GDB_LIMIT				0x9007

/* *** Color Bar (16 bit) *** */
#define TC358840_CB_CTL				0x7000
#define TC358840_CB_HSW				0x7008
#define TC358840_CB_VSW				0x700A
#define TC358840_CB_HTOTOAL			0x700C
#define TC358840_CB_VTOTOAL			0x700E
#define TC358840_CB_HACT			0x7010
#define TC358840_CB_VACT			0x7012
#define TC358840_CB_HSTART			0x7014
#define TC358840_CB_VSTART			0x7016

#endif /* __TC358840_REGS_H_ */
