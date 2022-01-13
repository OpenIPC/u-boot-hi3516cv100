#include <config.h>
#include <common.h>
#include <asm/io.h>
#include <asm/sizes.h>
#include <asm/arch/gpio.h>

#define HI3518_MUX_BASE 0x200F0000
#define HI3518_GPIO_BASE 0x20140000
#define HI3518_GPIO_BANK_SIZE 0x10000
#define HI3518_GPIO_DATA(bank) (HI3518_GPIO_BASE+(bank)*HI3518_GPIO_BANK_SIZE)
#define HI3518_GPIO_DIR(bank) (HI3518_GPIO_DATA(bank)+0x400)


struct  gpio_mux_def{
	uint32_t mux_reg;
	uint32_t gpio_val;
	uint32_t alt_val;
};

struct gpio_mux_def gpio_mux_18ev200[9][8] = {
{ // GPIO0
	{0x070,0,2}, // 0 (IR_IN,TEMPER_DQ)
	{0x074,0,2}, // 1 (TEMPER_DQ)
	{0x078,0,2}, // 2 (TEMPER_DQ)
	{0x030,0,1}, // 3 (SPI1_CSN1,VO_DATA0)
	{0x000,0,1}, // 4 (SENSOR_CLK)
	{0x004,1,0}, // 5 (SENSOR_RSTN)
	{0x008,0,3}, // 6 (FLASH_TRIG,SFC_EMMC_BOOT_MODE,SPI1_CSN1,VI_VS_BT1120)
	{0x00C,0,2}, // 7 (SHUTTER_TRIG,SFC_DEVICE_MODE,VI_HS_BT1120)
}, { // GPIO1
	{0x07C,0,1}, // 0 (VI_DATA13,I2S_BCLK_TX,PWM0)
	{0x080,0,1}, // 1 (VI_DATA10,I2S_SD_TX,UART1_TXD)
	{0x084,0,1}, // 2 (VI_DATA12,I2S_MCLK,UART1_CTSN)
	{0x088,0,1}, // 3 (VI_DATA11,I2S_WS_TX,UART2_RXD)
	{0x08C,0,1}, // 4 (VI_DATA15,VI_VS_SEN,I2S_WS_RX,UART1_RXD)
	{0x090,0,1}, // 5 (VI_DATA14,VI_HS_SEN,I2S_BCLK_RX,UART1_RTSN)
	{0x094,0,1}, // 6 (VI_DATA9,I2S_SD_RX,UART2_TXD)
	{0x098,0,1}, // 7 (SDIO0_CARD_POWER_EN)
}, { // GPIO2
	{0x010,0,1}, // 0 (RMII_CLK,VO_CLK,SDIO1_CCLK_OUT)
	{0x014,0,1}, // 1 (RMII_TX_EN,VO_VS,SDIO1_CARD_DETECT)
	{0x018,0,1}, // 2 (RMII_TXD0,VO_DATA5,SDIO1_CWPR)
	{0x01C,0,1}, // 3 (RMII_TDX1,VO_DE,SDIO1_CDATA1)
	{0x020,0,1}, // 4 (RMII_RX_DV,VO_DATA7,SDIO1_CDATA0)
	{0x024,0,1}, // 5 (RMII_RXD0,VO_DATA2,SDIO1_CDATA3)
	{0x028,0,1}, // 6 (RMII_RXD1,VO_DATA3,SDIO1_CCMD)
	{0x02C,0,1}, // 7 (EPHY_RST,BOOT_SEL,VO_HS,SDIO1_CARD_POWER_EN)
}, { // GPIO3
	{0x034,0,1}, // 0 (EPHY_CLK,VO_DATA1,SDIO1_CDATA2)
	{0x038,0,1}, // 1 (MDCK,BOOTROM_SEL,VO_DATA6)
	{0x03C,0,1}, // 2 (MDIO,VO_DATA4)
	{0x040,0,1}, // 3 (SPI0_SCLK,I2C0_SCL)
	{0x044,0,1}, // 4 (SPI0_SDO,I2C0_SDA)
	{0x048,0,1}, // 5 (SPI0_SDI)
	{0x04C,0,1}, // 6 (SPI0_CSN)
	{0x050,0,1}, // 7 (SPI1_SCLK,I2C1_SCL)
}, { // GPIO4
	{0x054,0,1}, // 0 (SPI1_SDO,I2C1_SDA)
	{0x058,0,1}, // 1 (SPI1_SDI)
	{0x05C,0,1}, // 2 (SPI1_CSN0)
	{0x060,0,1}, // 3 (I2C2_SDA)
	{0x064,0,1}, // 4 (I2C2_SCL)
	{0x068,0,1}, // 5 (USB_OVRCUR)
	{0x06C,0,1}, // 6 (USB_PWREN)
	{0x09C,0,1}, // 7 (SDIO0_CARD_DETECT)
}, { // GPIO5
	{0x0A0,0,1}, // 0 (SDIO0_CWPR)
	{0x0A4,0,1}, // 1 (SDIO0_CCLK_OUT)
	{0x0A8,0,1}, // 2 (SDIO0_CCMD)
	{0x0AC,0,1}, // 3 (SDIO0_CDATA0)
	{0x0B0,2,1}, // 4 (TEST_CLK,SDIO0_CDATA1)
	{0x0B4,0,1}, // 5 (SDIO0_CDATA2)
	{0x0B8,0,1}, // 6 (SDIO0_CDATA3)
	{0x0BC,0,2}, // 7 (EMMC_DATA6,I2S_SD_TX,UART1_RTSN)
}, { // GPIO6
	{0x0C0,0,2}, // 0 (EMMC_DATA5,I2S_WS_TX,UART1_RXD)
	{0x0C4,0,2}, // 1 (EMMC_DATA7,I2S_MCLK,UART1_CTSN)
	{0x0C8,0,2}, // 2 (EMMC_DS,I2S_SD_RX,UART1_TXD)
	{0x0CC,0,3}, // 3 (EMMC_DATA1,UART2_RXD)
	{0x0D0,0,3}, // 4 (EMMC_DATA2,I2S_BCLK_TX,UART2_TXD)
	{0x0D4,0,2}, // 5 (JTAG_TRSTN,SPI1_CSN1,I2S_MCLK)
	{0x0D8,0,2}, // 6 (JTAG_TCK,SPI1_SCLK,I2S_WS_TX,I2C1_SCL)
	{0x0DC,0,2}, // 7 (JTAG_TMS,SPI1_CSN0,I2S_SD_TX)
}, { // GPIO7
	{0x0E0,0,2}, // 0 (JTAG_TDO,SPI1_SDO,I2S_SD_RX,I2C1_SDA)
	{0x0E4,0,2}, // 1 (JTAG_TDI,SPI1_SDI,I2S_BCLK_TX)
	{0x0E8,1,2}, // 2 (PMC_PWM,PWM0)
	{0x0EC,1,0}, // 3 (PWM1)
	{0x0F0,1,2}, // 4 (PWM2,SPI1_CSN1)
	{0x0F4,1,0}, // 5 (PWM3)
	{0x0F8,1,0}, // 6 (SAR_ADC_CH0)
	{0x0FC,1,0}, // 7 (SAR_ADC_CH1)
}, { // GPIO8
	{0x100,1,0}, // 0 (SAR_ADC_CH2)
	{0x104,1,0}, // 1 (SAR_ADC_CH3)
	{0xFF0,0,1}, // 2 ()
	{0xFF0,0,1}, // 3 ()
	{0xFF0,0,1}, // 4 ()
	{0xFF0,0,1}, // 5 ()
	{0xFF0,0,1}, // 6 ()
	{0xFF0,0,1}, // 7 ()
}
};

struct gpio_mux_def gpio_mux[12][8] = { // 3516Cv100
{ // GPIO0
	{0x120,0,2}, // 0 ()
	{0x124,0,2}, // 1 ()
	{0x128,0,2}, // 2 ()
	{0x12C,0,1}, // 3 ()
	{0x130,0,1}, // 4 ()
	{0x134,1,0}, // 5 ()
	{0x138,0,1}, // 6 ()
	{0x13C,1,0}, // 7 ()
}, { // GPIO1
	{0x000,0,1}, // 0 ()
	{0x004,0,1}, // 1 ()
	{0x008,0,1}, // 2 ()
	{0x070,0,3}, // 3 ()
	{0x00C,0,1}, // 4 ()
	{0x010,0,1}, // 5 ()
	{0x014,0,1}, // 6 ()
	{0x07C,0,1}, // 7 ()
}, { // GPIO2
	{0x018,0,1}, // 0 ()
	{0x01C,0,1}, // 1 ()
	{0x020,0,1}, // 2 ()
	{0x024,0,1}, // 3 ()
	{0x028,0,1}, // 4 ()
	{0x02C,0,1}, // 5 ()
	{0x068,0,1}, // 6 ()
	{0x06C,0,1}, // 7 ()
}, { // GPIO3
	{0x030,0,1}, // 0 ()
	{0x034,0,1}, // 1 ()
	{0x058,0,1}, // 2 ()
	{0x05C,0,1}, // 3 ()
	{0x060,0,1}, // 4 ()
	{0x064,0,1}, // 5 ()
	{0x074,0,3}, // 6 ()
	{0x078,0,1}, // 7 ()
}, { // GPIO4
	{0x044,0,1}, // 0 ()
	{0x040,0,1}, // 1 ()
	{0x03C,0,1}, // 2 ()
	{0x038,0,1}, // 3 ()
	{0x054,0,1}, // 4 ()
	{0x050,0,1}, // 5 ()
	{0x04C,0,1}, // 6 ()
	{0x048,0,1}, // 7 ()
}, { // GPIO5
	{0x0B4,0,1}, // 0 ()
	{0x0B8,0,1}, // 1 ()
	{0x0BC,0,1}, // 2 ()
	{0x0C0,0,1}, // 3 ()
	{0x110,0,1}, // 4 ()
	{0x114,0,1}, // 5 ()
	{0x118,0,1}, // 6 ()
	{0x11C,0,1}, // 7 ()
}, { // GPIO6
	{0x080,0,1}, // 0 ()
	{0x084,0,1}, // 1 ()
	{0x088,0,1}, // 2 ()
	{0x08C,0,1}, // 3 ()
	{0x090,0,1}, // 4 ()
	{0x094,2,1}, // 5 ()
	{0x098,0,1}, // 6 ()
	{0x09C,0,1}, // 7 ()
}, { // GPIO7
	{0x0A0,0,1}, // 0 ()
	{0x0A4,0,1}, // 1 ()
	{0x0A8,1,2}, // 2 ()
	{0x0AC,0,1}, // 3 ()
	{0x0B0,1,0}, // 4 ()
	{0x0C4,1,0}, // 5 ()
	{0x108,0,1}, // 6 ()
	{0x10C,0,1}, // 7 ()
}, { // GPIO8
	{0xFF0,1,0}, // 0 ()
	{0xFF0,1,0}, // 1 ()
	{0xFF0,0,1}, // 2 ()
	{0xFF0,0,1}, // 3 ()
	{0xFF0,0,1}, // 4 ()
	{0xFF0,0,1}, // 5 ()
	{0xFF0,0,1}, // 6 ()
	{0xFF0,0,1}, // 7 ()
}, { // GPIO9
	{0x0C8,0,1}, // 0 ()
	{0x0CC,0,1}, // 1 ()
	{0x0D0,0,1}, // 2 ()
	{0x0D4,0,1}, // 3 ()
	{0x0D8,0,1}, // 4 ()
	{0x0DC,0,1}, // 5 ()
	{0x0E0,0,1}, // 6 ()
	{0x0E4,0,1}, // 7 ()
}, { // GPIO10
	{0x178,1,0}, // 0 ()
	{0x174,1,0}, // 1 ()
	{0x170,1,0}, // 2 ()
	{0x16C,1,0}, // 3 ()
	{0x168,1,0}, // 4 ()
	{0x164,1,0}, // 5 ()
	{0x160,1,0}, // 6 ()
	{0x15C,1,0}, // 7 ()
}, { // GPIO11
	{0x158,1,0}, // 0 ()
	{0x154,1,0}, // 1 ()
	{0x150,1,0}, // 2 ()
	{0x14C,1,0}, // 3 ()
	{0x148,1,0}, // 4 ()
	{0x144,1,0}, // 5 ()
	{0x140,1,0}, // 6 ()
	{0xFF0,0,1}, // 7 ()
}
};

int gpio_set_mode (int gpio,gpio_mode mode) {
	int pin = gpio % 10;
	int bank = gpio / 10;
	if (pin > 7 || bank > 8)
		return -1;
	writel ((mode==GPIO_MODE_ALT)?gpio_mux[bank][pin].alt_val:gpio_mux[bank][pin].gpio_val,HI3518_MUX_BASE + gpio_mux[bank][pin].mux_reg);
	uint32_t outmask = readl (HI3518_GPIO_DIR(bank));
	if (mode == GPIO_MODE_OUTPUT)
		outmask |= 1<<pin;
	if (mode == GPIO_MODE_INPUT)
		outmask &= ~(1<<pin);
	return writel (outmask,HI3518_GPIO_DIR(bank));
}

int gpio_get_mode (int gpio,gpio_mode *mode) {
    int pin = gpio % 10;
    int bank = gpio / 10;
    if (pin > 7 || bank > 8 || !mode)
        return -1;
    if (readl (HI3518_MUX_BASE + gpio_mux[bank][pin].mux_reg) != gpio_mux[bank][pin].gpio_val) {
        *mode = GPIO_MODE_ALT;
        return 0;
    }
    *mode = readl (HI3518_GPIO_DIR(bank)) & (1 << pin) ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT;
    return 0;
}

int gpio_set (int gpio,int value) {
	int pin = gpio % 10;
	int bank = gpio / 10;
	if (pin > 7 || bank > 8)
		return -1;
	return writel (value<<pin,HI3518_GPIO_DATA(bank) + (1<<pin)*4);
}

int gpio_get (int gpio) {
	int pin = gpio % 10;
	int bank = gpio / 10;
	if (pin > 7 || bank > 8)
		return -1;
	return (readl (HI3518_GPIO_DATA (bank) + (1<<pin)*4) & (1<<pin))?1:0;
}
