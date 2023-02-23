/* Copyright (c) Igor Institute, Inc.
 *
 * Author: Trent Piepho <tpiepho@gmail.com>
 *
 * Originally based on list in Linux driver: 
 *     Copyright (C) 2020 Micro Crystal SA
 *     Alexandre Belloni <alexandre.belloni@bootlin.com>
 */

#pragma once

#define RV3032_HSEC			0x00
#define RV3032_SEC			0x01
#define RV3032_MIN			0x02
#define RV3032_HOUR			0x03
#define RV3032_WDAY			0x04
#define RV3032_DAY			0x05
#define RV3032_MONTH			0x06
#define RV3032_YEAR			0x07
#define RV3032_ALARM_MIN		0x08
#define RV3032_ALARM_HOUR		0x09
#define RV3032_ALARM_DAY		0x0A
#define RV3032_TIMER0			0x0B
#define RV3032_TIMER1			0x0C
#define RV3032_STATUS			0x0D
#define RV3032_TLSB			0x0E
#define RV3032_TMSB			0x0F
#define RV3032_CTRL1			0x10
#define RV3032_CTRL2			0x11
#define RV3032_CTRL3			0x12
#define RV3032_TS_CTRL			0x13
#define RV3032_CLK_IRQ			0x14
#define RV3032_EVI			0x15
#define RV3032_TLOW			0x16
#define RV3032_THIGH			0x17
#define RV3032_TSLOW_COUNT		0x18
#define RV3032_TSLOW_SEC		0X19
#define RV3032_TSHIGH_COUNT		0X1F
#define RV3032_TSHIGH_SEC		0X20
#define RV3032_EEPROM_ADDR		0x3D
#define RV3032_EEPROM_DATA		0x3E
#define RV3032_EEPROM_CMD		0x3F
#define RV3032_RAM1			0x40
#define RV3032_PMU			0xC0
#define RV3032_OFFSET			0xC1
#define RV3032_CLKOUT1			0xC2
#define RV3032_CLKOUT2			0xC3
#define RV3032_TREF0			0xC4
#define RV3032_TREF1			0xC5
#define RV3032_EEPROM_PW0		0xC6
#define RV3032_EEPROM_PW1		0xC7
#define RV3032_EEPROM_PW2		0xC8
#define RV3032_EEPROM_PW3		0xC9
#define RV3032_EEPROM_PW_EN		0xCA
#define RV3032_EEPROM_USER              0xCB

#define RV3032_STATUS_VLF		BIT(0)
#define RV3032_STATUS_PORF		BIT(1)
#define RV3032_STATUS_EVF		BIT(2)
#define RV3032_STATUS_AF		BIT(3)
#define RV3032_STATUS_TF		BIT(4)
#define RV3032_STATUS_UF		BIT(5)
#define RV3032_STATUS_TLF		BIT(6)
#define RV3032_STATUS_THF		BIT(7)

#define RV3032_TLSB_CLKF		BIT(1)
#define RV3032_TLSB_EEBUSY		BIT(2)
#define RV3032_TLSB_EEF			BIT(3)
#define RV3032_TLSB_TEMP		GENMASK(7, 4)

#define RV3032_CTRL1_EERD		BIT(2)
#define RV3032_CTRL1_TE			BIT(3)
#define RV3032_CTRL1_USEL		BIT(4)
#define RV3032_CTRL1_WADA		BIT(5) // In Linux driver, not in data sheet?
#define RV3032_CTRL1_TD_MSK		GENMASK(1, 0)
#define RV3032_CTRL1_TD_4096		(0)
#define RV3032_CTRL1_TD_64		(1)
#define RV3032_CTRL1_TD_1		(2)
#define RV3032_CTRL1_TD_1_60		(3)

#define RV3032_CTRL2_STOP		BIT(0)
#define RV3032_CTRL2_EIE		BIT(2)
#define RV3032_CTRL2_AIE		BIT(3)
#define RV3032_CTRL2_TIE		BIT(4)
#define RV3032_CTRL2_UIE		BIT(5)
#define RV3032_CTRL2_CLKIE		BIT(6)
#define RV3032_CTRL2_TSE		BIT(7) // In Linux driver, not in data sheet?
#define RV3032_CTRL2_ALLIE		GENMASK(6, 2)

#define RV3032_CTRL3_TLIE		BIT(0)
#define RV3032_CTRL3_THIE		BIT(1)
#define RV3032_CTRL3_TLE		BIT(2)
#define RV3032_CTRL3_THE		BIT(3)
#define RV3032_CTRL3_BSIE		BIT(4)

#define RV3032_TS_CTRL_TLR		BIT(3)
#define RV3032_TS_CTRL_THR		BIT(4)
#define RV3032_TS_CTRL_EVR		BIT(5)

#define RV3032_EVI_ESYN			BIT(0)
#define RV3032_EVI_EHL			BIT(6)
#define RV3032_EVI_CLKDE		BIT(7)

#define RV3032_PMU_TCM			GENMASK(1, 0)
#define RV3032_PMU_TCM_OFF		(0u << 0)
#define RV3032_PMU_TCM_1750mV		(1u << 0)	// In BSM_DSM mode, uses Vdd
#define RV3032_PMU_TCM_3000mV		(2u << 0)
#define RV3032_PMU_TCM_4400mV		(3u << 0)
#define RV3032_PMU_TCR			GENMASK(3, 2)
#define RV3032_PMU_TCR_SHIFT		2
#define RV3032_PMU_TCR_600		(0u << RV3032_PMU_TCR_SHIFT)
#define RV3032_PMU_TCR_2k		(1u << RV3032_PMU_TCR_SHIFT)
#define RV3032_PMU_TCR_7k		(2u << RV3032_PMU_TCR_SHIFT)
#define RV3032_PMU_TCR_12k		(3u << RV3032_PMU_TCR_SHIFT)
#define RV3032_PMU_BSM_MSK		GENMASK(5, 4)
#define RV3032_PMU_BSM_SHIFT		4
#define RV3032_PMU_BSM_DIS		(0u << RV3032_PMU_BSM_SHIFT)
#define RV3032_PMU_BSM_DSM		(1u << RV3032_PMU_BSM_SHIFT)
#define RV3032_PMU_BSM_LSM		(2u << RV3032_PMU_BSM_SHIFT)
#define RV3032_PMU_NCLKE		BIT(6)

#define RV3032_OFFSET_MSK		GENMASK(5, 0)

#define RV3032_CLKOUT2_HFD_MSK		GENMASK(4, 0)
#define RV3032_CLKOUT2_FD_MSK		GENMASK(6, 5)
#define RV3032_CLKOUT2_FD_SHIFT		5
#define RV3032_CLKOUT2_FD_32768Hz	(0u << RV3032_CLKOUT2_FD_SHIFT)
#define RV3032_CLKOUT2_FD_1024Hz	(1u << RV3032_CLKOUT2_FD_SHIFT)
#define RV3032_CLKOUT2_FD_64Hz		(2u << RV3032_CLKOUT2_FD_SHIFT)
#define RV3032_CLKOUT2_FD_1Hz		(3u << RV3032_CLKOUT2_FD_SHIFT)
#define RV3032_CLKOUT2_OS		BIT(7)

#define RV3032_EEPROM_CMD_UPDATE	0x11
#define RV3032_EEPROM_CMD_WRITE		0x21
#define RV3032_EEPROM_CMD_READ		0x22

struct clock_regs {
	uint8_t hsec;
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
};

struct cal_regs {
	uint8_t weekday;
	uint8_t day;
	uint8_t month;
	uint8_t year;
};

struct alarm_regs {
	uint8_t min;
	uint8_t hour;
	uint8_t day;
};

struct time_regs {
	struct clock_regs clock;
	struct cal_regs cal;
};

struct eeprom_regs {
	uint8_t addr;
	uint8_t data;
	uint8_t cmd;
};
