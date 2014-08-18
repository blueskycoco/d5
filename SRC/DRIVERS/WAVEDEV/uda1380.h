#ifndef __LPC3250_UDA1380__
#define __LPC3250_UDA1380__

// General setup of the UDA1380 device with the Phytec board
// Connected to         : I2S1
// WS clock rate        : 8KHz to 48KHz
// Data width           : 16 bits
// Number of channels   : 2
// Bit rate clocking    : from LPC3250
// CODEC internal clock : UDA1380 WSPLL (x WS clock rate)
// Control to I2s       : I2C (via LPC32XX I2C1 channel)

#define UDA1380_REG_EVALCLK	0x00
#define UDA1380_REG_I2S		0x01
#define UDA1380_REG_PWRCTRL	0x02
#define UDA1380_REG_ANAMIX	0x03
#define UDA1380_REG_HEADAMP	0x04
#define UDA1380_REG_MSTRVOL	0x10
#define UDA1380_REG_MIXVOL	0x11
#define UDA1380_REG_MODEBBT	0x12
#define UDA1380_REG_MSTRMUTE  0x13
#define UDA1380_REG_MIXSDO	  0x14
#define UDA1380_REG_DECVOL	  0x20
#define UDA1380_REG_PGA		  0x21
#define UDA1380_REG_ADC		  0x22
#define UDA1380_REG_AGC		  0x23
#define UDA1380_REG_L3		  0x7f
#define UDA1380_REG_HEADPHONE 0x18
#define UDA1380_REG_DEC		  0x28

// UDA1380_REG_EVALCLK bit defines
#define EVALCLK_ADC_EN           0x0800  // Enable ADC clock
#define EVALCLK_DEC_EN           0x0400  // Enable decimator clock
#define EVALCLK_DAC_EN           0x0200  // Enable DAC clock
#define EVALCLK_INT_EN           0x0100  // Enable interpolator clock
#define EVALCLK_ADC_SEL_SYSCLK   0x0020  // Select SYSCLK input for ADC clock
#define EVALCLK_ADC_SEL_WSPLL    0x0000  // Select WSPLL clock for ADC clock
#define EVALCLK_DAC_SEL_SYSCLK   0x0010  // Select SYSCLK input for DAC clock
#define EVALCLK_DAC_SEL_WSPLL    0x0000  // Select WSPLL clock for DAC clock
#define EVALCLK_SYSDIV_SEL(n)    ((n) << 2) // System clock input divider select
#define EVALCLK_WSPLL_SEL6_12K   0x0000  // WSPLL input freq selection = 6.25 to 12.5K
#define EVALCLK_WSPLL_SEL12_25K  0x0001  // WSPLL input freq selection = 12.5K to 25K
#define EVALCLK_WSPLL_SEL25_50K  0x0002  // WSPLL input freq selection = 25K to 50K
#define EVALCLK_WSPLL_SEL50_100K 0x0003  // WSPLL input freq selection = 50K to 100K

// UDA1380_REG_PWRCTRL bit defines 
#define PWR_PON_PLL_EN           0x8000  // WSPLL enable
#define PWR_PON_HP_EN            0x2000  // Headphone driver enable
#define PWR_PON_DAC_EN           0x0400  // DAC power enable
#define PWR_PON_BIAS_EN          0x0100  // Power on bias enable (for ADC, AVC, and FSDAC)
#define PWR_EN_AVC_EN            0x0080  // Analog mixer enable
#define PWR_PON_AVC_EN           0x0040  // Analog mixer power enable
#define PWR_EN_LNA_EN            0x0010  // LNA and SDC power enable
#define PWR_EN_PGAL_EN           0x0008  // PGA left power enable
#define PWR_EN_ADCL_EN           0x0004  // ADC left power enable
#define PWR_EN_PGAR_EN           0x0002  // PGA right power enable
#define PWR_EN_ADCR_EN           0x0001  // ADC right power enable

// UDA1380_REG_MSTRMUTE bit defines
#define MSTRMUTE_MTM_MUTE_EN     0x4000  // Master mute enable

// UDA1380_REG_MODEBBT bit defines
#define MODEBBT_BOOST_FLAT       0x0000  // Bits for selecting flat boost
#define MODEBBT_BOOST_FULL       0xC000  // Bits for selecting maximum boost
#define MODEBBT_BOOST_MASK       0xC000  // Bits for selecting boost mask

#endif /* __LPC3250_UDA1380__ */
