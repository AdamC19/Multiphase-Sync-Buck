#ifndef _AD7172_H_
#define _AD7172_H_

#include "inttypes.h"
#include "stm32f3xx_hal.h"
#include "string.h"
#include "stdbool.h"

/* why do we have to do this */
typedef unsigned long uint32_t;

/* about 13.8 ns pause */
#define PAUSE_SHORT __asm__("nop")
#define PAUSE_25_NS __asm__("nop\nnop")
#define PAUSE_50_NS __asm__("nop\nnop\nnop\nnop")

#define AD7172_DFLT_OFFSET  0x800000
#define AD7172_DFLT_GAIN    0x555555
#define AD7172_FS           16777215

// REGISTER MAPPING
#define AD7172_COMMS_REG        0x00
#define AD7172_STATUS_REG       0x00
#define AD7172_ADCMODE_REG      0x01
#define AD7172_IFMODE_REG       0x02
#define AD7172_REGCHECK_REG     0x03
#define AD7172_DATA_REG         0x04
#define AD7172_GPIOCON_REG      0x06
#define AD7172_ID_REG           0x07
#define AD7172_CH0_REG          0x10
#define AD7172_CH1_REG          0x11
#define AD7172_CH2_REG          0x12
#define AD7172_CH3_REG          0x13
#define AD7172_SETUPCON0_REG    0x20
#define AD7172_SETUPCON1_REG    0x21
#define AD7172_SETUPCON2_REG    0x22
#define AD7172_SETUPCON3_REG    0x23
#define AD7172_FILTCON0         0x28
#define AD7172_FILTCON1         0x29
#define AD7172_FILTCON2         0x2A
#define AD7172_FILTCON3         0x2B
#define AD7172_OFFSET0          0x30
#define AD7172_OFFSET1          0x31
#define AD7172_OFFSET2          0x32
#define AD7172_OFFSET3          0x33
#define AD7172_GAIN0            0x38
#define AD7172_GAIN1            0x39
#define AD7172_GAIN2            0x3A
#define AD7172_GAIN3            0x3B


// COMMS REGISTER COMMANDS
/* indicates that the next operation is a read of the data register */
#define AD7172_CMD_READ     (1 << 6)
/* indicates that the next operation is a write to a specified register */
#define AD7172_CMD_WRITE    0x00


// STATUS REGISTER VALUES
#define AD7172_ADC_ERROR    (1 << 6)
#define AD7172_CRC_ERROR    (1 << 5)
#define AD7172_REG_ERROR    (1 << 4)
/* indicates which channel was active for the last ADC conversion */
#define AD7172_CH0_ACTIVE   0x00
#define AD7172_CH1_ACTIVE   0x01
#define AD7172_CH2_ACTIVE   0x02
#define AD7172_CH3_ACTIVE   0x03


// ADC MODE REGISTER VALUES
/* Enables internal reference and outputs buffered 2.5V to REFOUT pin */
#define AD7172_REF_EN               (1 << 15)
#define AD7172_HIDE_DELAY_DISABLE   (1 << 14)
/* when only single channel is active, ADC will only output at the settled filter data rate */
#define AD7172_SING_CYC             (1 << 13)
/*  */
#define AD7172_DELAY_32US           (0x01 << 8)
/*  */
#define AD7172_DELAY_128US          (0x02 << 8)
/*  */
#define AD7172_DELAY_320US          (0x03 << 8)
/*  */
#define AD7172_DELAY_800US          (0x04 << 8)
/*  */
#define AD7172_DELAY_1600US         (0x05 << 8)
/*  */
#define AD7172_DELAY_4000US         (0x06 << 8)
/*  */
#define AD7172_DELAY_8000US         (0x07 << 8)
/* Continuous conversion mode */
#define AD7172_MODE_CONT_CONV       (0x00 << 4)
/*  */
#define AD7172_MODE_SINGLE_CONV     (0x01 << 4)
/*  */
#define AD7172_MODE_STANDBY         (0x02 << 4)
/*  */
#define AD7172_MODE_PWR_DOWN        (0x03 << 4)
/*  */
#define AD7172_MODE_INT_OFFSET_CAL  (0x04 << 4)
/*  */
#define AD7172_MODE_SYS_OFFSET_CAL  (0x06 << 4)
/*  */
#define AD7172_MODE_SYS_GAIN_CAL    (0x07 << 4)
/* Selects and enables internal oscillator */
#define AD7172_CLOCKSEL_INTERNAL    (0x00 << 2)
/* Enables internal oscillator output on XTAL2/CLKIO pin */
#define AD7172_CLOCKSEL_INT_OUTPUT  (0x01 << 2)
/* Clock input on the XTAL2/CLKIO pin */
#define AD7172_CLOCKSEL_EXT_CLK     (0x02 << 2)
/* Enables external crystal on XTAL1 and XTAL2/CLKIO pin */
#define AD7172_CLOCKSEL_EXT_XTAL    (0x03 << 2)


// IFMODE REGISTER COMMANDS
/* Allows use of SYNC/ERROR pin as a contorl for conversions when cycling channels  */
#define AD7172_ALT_SYNC     (1 << 12)
/* Enable higher drive strength of DOUT/RDY pin, for low IOVDD supply and moderate bus capacitance */
#define AD7172_IOSTRENGTH   (1 << 11)
/*  */
#define AD7172_DOUT_RESET   (1 << 8)
/* Enables continuous read of the DATA register (must be in cont. conv. mode) */
#define AD7172_CONTREAD     (1 << 7)
/* Enable status reg to be appened to data reg so channel and status info are sent with data */
#define AD7172_DATA_STAT    (1 << 6)
/* Enables register integ checker */
#define AD7172_REG_CHECK    (1 << 5)
/* Enables XOR checksum for reg reads. Reg writes still use CRC with this set */
#define AD7172_CRC_EN_XOR   (0x01 << 2)
/* Enables CRC checksum for read and write transactions */
#define AD7172_CRC_EN_CRC   (0x02 << 2)


// ID REGISTER
/* A 16-bit code that is specific to the ADC */
#define AD7172_ID_MASK   0x00DF

// CHANNEL REGISTER LAYOUT
/* Enables this channel */
#define AD7172_CH_EN           (1 << 15)
/* Use Setup 0 for this channel */
#define AD7172_SETUP_SEL_0     (0x00 << 12)
/* Use Setup 1 for this channel */
#define AD7172_SETUP_SEL_1     (0x01 << 12)
/* Use Setup 2 for this channel */
#define AD7172_SETUP_SEL_2     (0x02 << 12)
/* Use Setup 3 for this channel */
#define AD7172_SETUP_SEL_3     (0x03 << 12)
/* Input AIN0 connected to positive input (default) */
#define AD7172_AINPOS_AIN0     (0x00 << 5)
/* Input AIN1 connected to positive input */
#define AD7172_AINPOS_AIN1     (0x01 << 5)
/* Input AIN2 connected to positive input */
#define AD7172_AINPOS_AIN2     (0x02 << 5)
/* Input AIN3 connected to positive input */
#define AD7172_AINPOS_AIN3     (0x03 << 5)
/* Input AIN4 connected to positive input */
#define AD7172_AINPOS_AIN4     (0x04 << 5)
/*  */
#define AD7172_AINPOS_TEMP_P   (0x11 << 5)
/*  */
#define AD7172_AINPOS_TEMP_N   (0x12 << 5)
/*  */
#define AD7172_AINPOS_ONE_FIFTH_SUPPLY_P (0x13 << 5)
/*  */
#define AD7172_AINPOS_ONE_FIFTH_SUPPLY_N (0x14 << 5)
/*  */
#define AD7172_AINPOS_REF_P    (0x15 << 5)
/*  */
#define AD7172_AINPOS_REF_N    (0x16 << 5)
/*  */
#define AD7172_AINNEG_AIN0     (0x00)
/*  */
#define AD7172_AINNEG_AIN1     (0x01)
/*  */
#define AD7172_AINNEG_AIN2     (0x02)
/*  */
#define AD7172_AINNEG_AIN3     (0x03)
/*  */
#define AD7172_AINNEG_AIN4     (0x04)
/*  */
#define AD7172_AINNEG_TEMP_P   (0x11)
/*  */
#define AD7172_AINNEG_TEMP_N   (0x12)
/*  */
#define AD7172_AINNEG_ONE_FIFTH_SUPPLY_P   (0x13)
/*  */
#define AD7172_AINNEG_ONE_FIFTH_SUPPLY_N   (0x14)
/*  */
#define AD7172_AINNEG_REF_P    (0x15)
/*  */
#define AD7172_AINNEG_REF_N    (0x16)


// SETUP CONFIGURATION REGISTER LAYOUT
/* Sets output coding of the ADC as unipolar coded output */
#define AD7172_UNIPOLAR   (0 << 12)
/* Sets output coding of the ADC as bipolar coded output (offset binary) */
#define AD7172_BIPOLAR    (1 << 12)
/* Enables the REF+ input buffer */
#define AD7172_REFBUF_P_EN      (1 << 11)
/* Enables the REF- input buffer */
#define AD7172_REFBUF_N_EN      (1 << 10)
/*  */
#define AD7172_AINBUF_P_EN      (1 << 9)
/*  */
#define AD7172_AINBUF_N_EN      (1 << 8)
/* enables a 10uA current source on the inputs selected. If input is open, output goes full scale */
#define AD7172_BURNOUT_EN       (1 << 7)
/* Enables external reference */
#define AD7172_REF_SEL_EXT      (0x00 << 4)
/* Enables internal 2.5V reference */
#define AD7172_REF_SEL_INT_2V5  (0x02 << 4)
/* Enables AVDD1-AVSS as the reference. Can be used as a diagnostic */
#define AD7172_REF_SEL_SUPPLY   (0x03 << 4)


// FILTER CONFIGURATION LAYOUT
/* If set, mapping of filter reg changes to directly program decimation rate of sinc3 filter (see datasheet for more) */
#define AD7172_SINC3_MAP        (1 << 15)
/* Enables various postfilters for enhanced 50 & 60Hz rejection. ORDER bits must be set to select sinc5+sinc1. */
#define AD7172_ENHFILTEN        (1 << 11)
/*  */
#define AD7172_ENHFILT_27SPS_47DB   (0x02 << 8)
/*  */
#define AD7172_ENHFILT_21SPS25_62DB (0x03 << 8)
/*  */
#define AD7172_ENHFILT_20SPS_86DB   (0x05 << 8)
/*  */
#define AD7172_ENHFILT_16SPS67_92DB (0x06 << 8)
/*  */
#define AD7172_ORDER_SINC5_SINC1    (0x00 << 5)
/*  */
#define AD7172_ORDER_SINC3          (0x03 << 5)
/*  */
#define AD7172_ODR_1007             0x0A
/*  */
#define AD7172_ODR_15625            (0x06)
/*  */
#define AD7172_ODR_31250            0x0000


/* mask to allow only the highest 20 (noise-free) bits */
#define AD7172_20_BIT_MASK          0x00FFFFF0


typedef struct AD7172 {
    SPI_HandleTypeDef * hspi;
    GPIO_TypeDef * cs_port;
    uint16_t cs_pin;
    uint8_t status;
    uint16_t id;
    uint16_t ch_configs[4];
    uint16_t setups[4];
    uint16_t filters[4];
    uint16_t adc_mode_config;
    uint16_t adc_if_mode_config;
    uint32_t conversions[4];

} AD7172_TypeDef;

uint32_t ad7172_conv_to_mv(uint32_t conv);
float ad7172_conv_to_voltage(float conv);

bool ad7172_config_if_mode(AD7172_TypeDef* adc, uint16_t config);
bool ad7172_config_adc_mode(AD7172_TypeDef* adc, uint16_t config);
bool ad7172_config_channel(AD7172_TypeDef* adc, int ch_num, uint16_t config);
bool ad7172_config_setup(AD7172_TypeDef* adc, int setup_num, uint16_t config);
bool ad7172_config_filter(AD7172_TypeDef* adc, int filter_num, uint16_t config);
bool ad7172_calibrate(AD7172_TypeDef* adc);

bool ad7172_disable_channel(AD7172_TypeDef* adc, int ch_num);

bool ad7172_enable_channel(AD7172_TypeDef* adc, int ch_num);

bool ad7172_reset(AD7172_TypeDef* adc);

bool ad7172_check_rdy(AD7172_TypeDef* adc, int timeout_us);

bool ad7172_read_conv(AD7172_TypeDef* adc, int timeout_us);

bool ad7172_read_data(AD7172_TypeDef* adc);

bool ad7172_check_id(AD7172_TypeDef* adc);

uint16_t ad7172_read_reg16(AD7172_TypeDef* adc, uint8_t reg);
bool ad7172_write_reg16(AD7172_TypeDef* adc, uint8_t reg, uint16_t data);

bool ad7172_read_reg(AD7172_TypeDef* adc, uint8_t reg, uint8_t* data, int size);

bool ad7172_write_reg(AD7172_TypeDef* adc, uint8_t reg, uint8_t* data, int size);

void delay_us();

#endif
