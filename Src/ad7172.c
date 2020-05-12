#include "ad7172.h"

uint32_t ad7172_conv_to_mv(uint32_t conv){
	return (uint32_t)(250*conv)/( AD7172_FS/10 );
}

float ad7172_conv_to_voltage(float conv){
	return (float)((2.5*conv)/AD7172_FS);
}

/**
 * 
 */
bool ad7172_config_if_mode(AD7172_TypeDef* adc, uint16_t config){
	adc->adc_if_mode_config = config;
	return ad7172_write_reg16(adc, AD7172_IFMODE_REG, adc->adc_if_mode_config);
}

/**
 * 
 */
bool ad7172_config_adc_mode(AD7172_TypeDef* adc, uint16_t config){
	adc->adc_mode_config = config;
	return ad7172_write_reg16(adc, AD7172_ADCMODE_REG, adc->adc_mode_config);
}

/**
 * @brief sets the configuration of a channel to config
 */
bool ad7172_config_channel(AD7172_TypeDef* adc, int ch_num, uint16_t config){
	uint8_t reg;
	switch(ch_num){
		case 0:{
			reg = AD7172_CH0_REG;
			break;}
		case 1:{
			reg = AD7172_CH1_REG;
			break;}
		case 2:{
			reg = AD7172_CH2_REG;
			break;}
		case 3:{
			reg = AD7172_CH3_REG;
			break;}
		default:{
			return false;}
	}

	adc->ch_configs[ch_num] = config;

	return ad7172_write_reg16(adc, reg, config);
}

/**
 * @brief writes config to the setup indicated by setup_num (0 - 3)
 * @return true if successful, false if setup_num out of bounds or write adc failed
 */
bool ad7172_config_setup(AD7172_TypeDef* adc, int setup_num, uint16_t config){
	uint8_t reg;
	switch (setup_num)
	{
	case 0:{
		reg = AD7172_SETUPCON0_REG;
		break;}
	case 1:{
		reg = AD7172_SETUPCON1_REG;
		break;}
	case 2:{
		reg = AD7172_SETUPCON2_REG;
		break;}
	case 3:{
		reg = AD7172_SETUPCON3_REG;
		break;}
	default:{
		return false;}
	}
	adc->setups[setup_num] = config;
	return ad7172_write_reg16(adc, reg, config);
}

/**
 * @brief writes config to the filter conf reg indicated by setup_num (0 - 3)
 * @return true if successful, false if setup_num out of bounds or write adc failed
 */
bool ad7172_config_filter(AD7172_TypeDef* adc, int filter_num, uint16_t config){
	uint8_t reg;
	switch (filter_num)
	{
	case 0:{
		reg = AD7172_FILTCON0;
		break;}
	case 1:{
		reg = AD7172_FILTCON1;
		break;}
	case 2:{
		reg = AD7172_FILTCON2;
		break;}
	case 3:{
		reg = AD7172_FILTCON3;
		break;}
	default:{
		return false;}
	}
	adc->filters[filter_num] = config;
	return ad7172_write_reg16(adc, reg, config);
}

/**
 * 
 */
bool ad7172_calibrate(AD7172_TypeDef* adc){
	bool retval = true;
	// start with all channels disabled
	ad7172_disable_channel(adc, 0);
	ad7172_disable_channel(adc, 1);
	ad7172_disable_channel(adc, 2);
	ad7172_disable_channel(adc, 3);

	// go through and calibrate each channel
	for(int i = 0; i < 4; i++){
		uint16_t og_filter_conf = adc->filters[i]; // remember initial filter config for later
		ad7172_config_filter(adc, i, AD7172_ODR_1007); // lower the data rate
		ad7172_enable_channel(adc, i);

		uint16_t mode = ad7172_read_reg16(adc, AD7172_ADCMODE_REG);
		uint16_t new_mode = mode & ~(0x07 << 4); // clear the mode bits 6:4

		new_mode |= AD7172_MODE_INT_OFFSET_CAL; // start with internal offset
		ad7172_write_reg16(adc, AD7172_ADCMODE_REG, new_mode);
		delay_us();
		retval = ad7172_check_rdy(adc, 1500) && retval;

		new_mode = new_mode & ~(0x07 << 4); // clear the mode bits 6:4
		new_mode |= AD7172_MODE_SYS_OFFSET_CAL; // system offset
		ad7172_write_reg16(adc, AD7172_ADCMODE_REG, new_mode);
		delay_us();
		retval = ad7172_check_rdy(adc, 1500) && retval;

		new_mode = new_mode & ~(0x07 << 4); // clear the mode bits 6:4
		new_mode |= AD7172_MODE_SYS_GAIN_CAL; // system gain calibration
		ad7172_write_reg16(adc, AD7172_ADCMODE_REG, new_mode);
		delay_us();
		retval = ad7172_check_rdy(adc, 1500) && retval;

		ad7172_write_reg16(adc, AD7172_ADCMODE_REG, mode); // write the original mode config

		ad7172_disable_channel(adc, i);
		ad7172_config_filter(adc, i, og_filter_conf); // reconfigure the filter to whatever it was
	}
	return retval;
}

/**
 * @brief clears the enable bit for the specified channel
 */
bool ad7172_disable_channel(AD7172_TypeDef* adc, int ch_num){
	bool retval = true;
	uint8_t reg;
	switch(ch_num){
		case 0:{
			reg = AD7172_CH0_REG;
			break;}
		case 1:{
			reg = AD7172_CH1_REG;
			break;}
		case 2:{
			reg = AD7172_CH2_REG;
			break;}
		case 3:{
			reg = AD7172_CH3_REG;
			break;}
		default:{
			return false;}
	}
	uint8_t data[2];
	retval = retval && ad7172_read_reg(adc, reg, data, 2); // read existing config
	uint16_t ch_conf = (data[0] << 8) | data[1]; // 
	ch_conf &= ~AD7172_CH_EN; // clear the channel enable bit
	data[0] = ch_conf >> 8;
	data[1] = ch_conf & 0xFF;
	retval = retval && ad7172_write_reg(adc, reg, data, 2);
	return retval;
}

/**
 * @brief sets the enable bit for the specified channel
 */
bool ad7172_enable_channel(AD7172_TypeDef* adc, int ch_num){
	bool retval = true; // must start as true
	uint8_t reg;
	switch(ch_num){
		case 0:{
			reg = AD7172_CH0_REG;
			break;}
		case 1:{
			reg = AD7172_CH1_REG;
			break;}
		case 2:{
			reg = AD7172_CH2_REG;
			break;}
		case 3:{
			reg = AD7172_CH3_REG;
			break;}
		default:{
			return false;}
	}
	uint8_t data[2];
	retval = retval && ad7172_read_reg(adc, reg, data, 2); // read existing config
	uint16_t ch_conf = (data[0] << 8) | data[1]; // 
	ch_conf |= AD7172_CH_EN; // set the channel enable bit
	data[0] = ch_conf >> 8;
	data[1] = ch_conf & 0xFF;
	retval = retval && ad7172_write_reg(adc, reg, data, 2);
	return retval;
}

/**
 * @brief resets device by sending 64 clock cycles with MOSI high (0xff)
 * @return true if spi activity was successful, false if not
 */
bool ad7172_reset(AD7172_TypeDef* adc){
	uint8_t empty[8];
	memset(empty, 0x00, 8);
	
    HAL_GPIO_WritePin(adc->cs_port, adc->cs_pin, GPIO_PIN_RESET); // bring cs low
    PAUSE_25_NS;
	HAL_StatusTypeDef hal_status_tx = HAL_SPI_Transmit(adc->hspi, empty, 8, 100);
    HAL_GPIO_WritePin(adc->cs_port, adc->cs_pin, GPIO_PIN_SET); // bring cs low

	return (hal_status_tx == HAL_OK);
}

/**
 * Brings the cs line low and reads the state of the MISO line until it goes low or timeout occurs
 * @param adc pointer to the AD7172 struct
 * @param timeout_us timeout in microseconds
 * @return true if a conversion is ready, false if not or if timeout occurs
 */
bool ad7172_check_rdy(AD7172_TypeDef* adc, int timeout_us){
    bool retval = false;
    int t = 0;
    HAL_GPIO_WritePin(adc->cs_port, adc->cs_pin, GPIO_PIN_RESET); // bring cs low
    PAUSE_50_NS;
    do{
        retval = (HAL_GPIO_ReadPin(adc->hspi, adc->hspi) == GPIO_PIN_RESET);
        delay_us();
    }while(!retval && (++t) < timeout_us);

    HAL_GPIO_WritePin(adc->cs_port, adc->cs_pin, GPIO_PIN_SET); // bring cs HIGH
    
    return retval;
}

/**
 * @brief waits for a conversion, stores result in the correct field of adc
 * @return false if no conversion was updated, true if we read a conversion
 */
bool ad7172_read_conv(AD7172_TypeDef* adc, int timeout_us){
	if(ad7172_check_rdy(adc, timeout_us)){
		bool ret = true;
		uint32_t result = 0;
		
		// check DATA_STAT bit
		if(adc->adc_if_mode_config & AD7172_DATA_STAT){
			// expect the status reg to follow the data reg
			uint8_t data[4];
			ret = ret && ad7172_read_reg(adc, AD7172_DATA_REG, data, 4);
			result = (data[0] << 16) | (data[1] << 8) | data[2];
			adc->status = data[3];
		}else{
			// first read just the data, then read the STATUS reg to figure out which channel was converted
			uint8_t data[3];
			ret = ret && ad7172_read_reg(adc, AD7172_DATA_REG, data, 3);
			result = (data[0] << 16) | (data[1] << 8) | data[2];
			ret = ret && ad7172_read_reg(adc, AD7172_STATUS_REG, &adc->status, 1);
		}
		
		if(ret){
			adc->conversions[adc->status & 0x03] = result;
		}

		return ret;

	}else{
		return false;
	}
}

/**
 * @brief attempts to read the ID register
 * @return true if ID reads back as expected, false if any error occurs
 */
bool ad7172_check_id(AD7172_TypeDef* adc){
	uint8_t data[2];
	bool retval = ad7172_read_reg(adc, AD7172_ID_REG, data, 2);
	adc->id = ((data[0] << 8) | data[1]);
	retval = retval && ((adc->id & ~AD7172_ID_MASK) == 0);
	return retval;
}

uint16_t ad7172_read_reg16(AD7172_TypeDef* adc, uint8_t reg){
	uint8_t data[2];
	if(ad7172_read_reg(adc, reg, data, 2)){
		return (uint16_t)((data[0] << 8) | data[1]);
	}else{
		return 0xFFFF;
	}
	
}

bool ad7172_write_reg16(AD7172_TypeDef* adc, uint8_t reg, uint16_t data){
	uint8_t dat[2];
	dat[0] = data >> 8;
	dat[1] = data & 0xFF;
	return ad7172_write_reg(adc, reg, dat, 2);
}

/**
 * @brief Reads size many bytes from specified register to data array.
 * @return true if tx and rx were successful, false if either was not
 */
bool ad7172_read_reg(AD7172_TypeDef* adc, uint8_t reg, uint8_t* data, int size){
    // first, write to the communications register
    uint8_t tx_data = AD7172_CMD_READ | reg;
    HAL_GPIO_WritePin(adc->cs_port, adc->cs_pin, GPIO_PIN_RESET); // bring cs low
    PAUSE_25_NS;
    HAL_StatusTypeDef hal_status_tx = HAL_SPI_Transmit(adc->hspi, &tx_data, 1, 5);
    PAUSE_25_NS;
    HAL_StatusTypeDef hal_status_rx = HAL_SPI_Receive(adc->hspi, data, size, 5);
    HAL_GPIO_WritePin(adc->cs_port, adc->cs_pin, GPIO_PIN_SET); // bring cs high

    return (hal_status_rx == HAL_OK) && (hal_status_tx == HAL_OK);
}

/**
 * @brief Writes size many bytes from data array to specified register.
 * @return true if tx and rx were successful, false if either was not
 */
bool ad7172_write_reg(AD7172_TypeDef* adc, uint8_t reg, uint8_t* data, int size){
    // first, write to the communications register
    uint8_t tx_data = AD7172_CMD_WRITE | reg;
    HAL_GPIO_WritePin(adc->cs_port, adc->cs_pin, GPIO_PIN_RESET); // bring cs low
    PAUSE_25_NS;
    HAL_StatusTypeDef hal_status_tx = HAL_SPI_Transmit(adc->hspi, &tx_data, 1, 5);
    // PAUSE_25_NS;
    HAL_StatusTypeDef hal_status_rx = HAL_SPI_Transmit(adc->hspi, data, size, 5);
    HAL_GPIO_WritePin(adc->cs_port, adc->cs_pin, GPIO_PIN_SET); // bring cs high

    return (hal_status_rx == HAL_OK) && (hal_status_tx == HAL_OK);
}

/**
 * Delay 1us, approximately. Assumes 72MHz operation
 */
void delay_us(){
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
	__asm__("nop");
}