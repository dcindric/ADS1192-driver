#include "ads1192.h"

// Global ADS119X configuration structure, accessed by most of the driver functions.
ads119x_config_t ads119x_config = {0};

/*Function for reading register values in the */
ads119x_ret_val_t ads119x_read_register(uint8_t reg_addr, uint8_t *reg_data)
{
    uint8_t reg_read_opcode[ADS119X_READ_OPCODE_LEN] = {0};

    reg_read_opcode[0] = ADS119X_READ_FIRST_OPCODE_BYTE | reg_addr;
    reg_read_opcode[1] = ADS119X_READ_SECOND_OPCODE_BYTE;

    ads119x_ret_val_t ret_val = ads119x_config.f_dev_spi_write(reg_read_opcode, ADS119X_READ_OPCODE_LEN);
    
    if (ADS119X_RET_SUCCESS == ret_val)
    {
        ret_val = ads119x_config.f_dev_spi_read(reg_data, 1);
    }

    return ret_val;   
}

ads119x_ret_val_t ads119x_write_register(uint8_t reg_addr, uint8_t reg_command)
{
    uint8_t reg_write_data[ADS119X_WRITE_OPCODE_LEN + 1] = {0};

    reg_write_data[0] = ADS119X_WRITE_FIRST_OPCODE_BYTE | reg_addr;
    reg_write_data[1] = ADS119X_WRITE_SECOND_OPCODE_BYTE;
    reg_write_data[2] = reg_command;

    uint8_t reg_current_status = 0;
    
    //Before writing into register, its current status is obtained in order to preserved bits that are not affected
    //by register command.
    ads119x_ret_val_t ret_val = ads119x_read_register(reg_addr, &reg_current_status);

    if (ADS119X_RET_SUCCESS == ret_val)
    {
        //Perform logical OR of register command and its current status.
        reg_write_data[2] |= reg_current_status;
        ret_val = ads119x_config.f_dev_spi_write(reg_write_data, sizeof(reg_write_data));     
    }

    return ret_val;
}

ads119x_ret_val_t ads119x_write_command(ads119x_command_t ads119x_command)
{
    uint8_t write_command = (uint8_t) ads119x_command;
    // Command size will always be equal to one byte.
    ads119x_ret_val_t ret_val = ads119x_config.f_dev_spi_write(&write_command, sizeof(write_command));

    return ret_val;
}


ads119x_ret_val_t ads119x_get_id_register(uint8_t *reg_id_data)
{
    ads119x_ret_val_t ret_val = ads119x_read_register(ADS119X_REG_ID, reg_id_data);

    return ret_val;
}

// CONFIG1 register
ads119x_ret_val_t ads119x_get_config_1_register(uint8_t *reg_config1_data)
{
    ads119x_ret_val_t ret_val = ads119x_read_register(ADS119X_REG_CONFIG1, reg_config1_data);

    return ret_val;
}

asd119x_conversion_mode_t ads119x_get_conversion_mode(const uint8_t reg_config1_data)
{
    asd119x_conversion_mode_t conversion_mode = ADS119X_CONV_MODE_CONT;

    // MSB contains information about conversion mode (0 - continous, 1 - single shot mode)
    if (reg_config1_data & 0x80)
    {
        conversion_mode = ADS119X_CONV_MODE_SINGLE;
    }
    else
    {
        conversion_mode = ADS119X_CONV_MODE_CONT;
    }

    return conversion_mode;
}

void ads119x_set_conversion_mode(asd119x_conversion_mode_t conv_mode)
{
    if (ADS119X_CONV_MODE_CONT == conv_mode)
    {
        ads119x_write_register (ADS119X_REG_CONFIG1, 0);
    }
    else if (ADS119X_CONV_MODE_SINGLE == conv_mode)
    {
        ads119x_write_register (ADS119X_REG_CONFIG1, 0x80);
    }
}

ads119x_data_rate_t ads119x_get_data_rate(const uint8_t reg_config1_data)
{
    //500 SPS is the default value.
    ads119x_data_rate_t data_rate = ADS119X_DATA_RATE_500_SPS;

    //3 LSBs contain data rate information.
    switch (reg_config1_data & 0x07)
    {
        case 0:
            data_rate = ADS119X_DATA_RATE_125_SPS;
        break;

        case 1:
            data_rate = ADS119X_DATA_RATE_250_SPS;
        break;

        case 2:
            data_rate = ADS119X_DATA_RATE_500_SPS;
        break;

        case 3:
            data_rate = ADS119X_DATA_RATE_1000_SPS;
        break;

        case 4:
            data_rate = ADS119X_DATA_RATE_2000_SPS;
        break;

        case 5:
            data_rate = ADS119X_DATA_RATE_4000_SPS;
        break;

        case 6:
            data_rate = ADS119X_DATA_RATE_8000_SPS;
        break;

        default:
            data_rate = ADS119X_DATA_RATE_INVALID;
        break;
    }

    return data_rate;
}

void ads119x_set_data_rate(ads119x_data_rate_t ads119x_data_rate)
{
    ads119x_write_register (ADS119X_REG_CONFIG1, (uint8_t) ads119x_data_rate);
}

//CONFIG2 register
ads119x_ret_val_t ads119x_get_config_2_register(uint8_t *reg_config2_data)
{
    ads119x_ret_val_t ret_val = ads119x_read_register(ADS119X_REG_CONFIG2, reg_config2_data);

    return ret_val;
}

void ads119x_lead_off_comparator_enable (void)
{
    //Writing 1 to bit 6 enables the comparator.
    ads119x_write_register (ADS119X_REG_CONFIG2, ENABLE << 6);
}

void ads119x_lead_off_comparator_disable (void)
{
    //Writing 0 to bit 6 disables the comparator.
    ads119x_write_register (ADS119X_REG_CONFIG2, DISABLE << 6);
}

void ads119x_reference_buffer_enable (void)
{
    //Writing 1 to bit 5 enables the reference buffer.
    ads119x_write_register (ADS119X_REG_CONFIG2, ENABLE << 5);
}
void ads119x_reference_buffer_disable (void)
{
    //Writing 0 to bit 5 disables the reference buffer.
    ads119x_write_register (ADS119X_REG_CONFIG2, DISABLE << 5);
}

void ads119x_set_volt_reference (ads119x_reference_t volt_reference_val)
{
    if (ADS119X_2_42V == volt_reference_val)
    {
        //Writing 0 to bit 4 sets voltage reference to 2.42V (default).
        ads119x_write_register (ADS119X_REG_CONFIG2, DISABLE << 4);
    }
    else if (ADS119X_4_033V == volt_reference_val)
    {
        //Writing 1 to bit 5 sets voltage reference to 4.033V.
        ads119x_write_register (ADS119X_REG_CONFIG2, ENABLE << 4);
    }
}

void ads119x_clock_osc_out_enable (void)
{
    //Writing 1 to bit 3 enables the oscillator clock output.
    ads119x_write_register (ADS119X_REG_CONFIG2, ENABLE << 3);
}

void ads119x_clock_osc_out_disable (void)
{
    //Writing 0 to bit 3 disables the oscillator clock output.
    ads119x_write_register (ADS119X_REG_CONFIG2, DISABLE << 3);
}

void ads119x_test_signal_enable (void)
{
    //Writing 1 to bit 1 enables the test signal.
    ads119x_write_register (ADS119X_REG_CONFIG2, ENABLE << 1);
}

void ads119x_test_signal_disable (void)
{
    //Writing 0 to bit 1 disables the test signal.
    ads119x_write_register (ADS119X_REG_CONFIG2, DISABLE << 1);
}

void ads119x_test_signal_set_freq_dc (void)
{
    //Writing 0 to bit 0 sets test signal frequency to 0 Hz (dc).
    ads119x_write_register (ADS119X_REG_CONFIG2, DISABLE << 0);
}

void ads119x_test_signal_set_freq_1hz (void)
{
    //Writing 1 to bit 0 sets test signal frequency to square wave at 1 Hz.
    ads119x_write_register (ADS119X_REG_CONFIG2, ENABLE << 0);
}



//LOFF register
ads119x_ret_val_t ads119x_get_loff_register(uint8_t *reg_loff_data)
{
     ads119x_ret_val_t ret_val = ads119x_read_register(ADS119X_REG_LOFF, reg_loff_data);

     return ret_val;
}


void ads119x_lead_off_comparator_set_pos_val (ads119x_lead_off_comp_pos_side_val_t comp_pos_val)
{
    ads119x_write_register (ADS119X_REG_LOFF, comp_pos_val);    
}
void ads119x_lead_off_comparator_set_neg_val (ads119x_lead_off_comp_neg_side_val_t comp_neg_val)
{
    ads119x_write_register (ADS119X_REG_LOFF, comp_neg_val);
}

void ads119x_lead_off_current_set_val (ads119x_lead_off_curr_val_t lead_off_curr_val)
{
    ads119x_write_register (ADS119X_REG_LOFF, lead_off_curr_val);
}
void ads119x_lead_off_freq_dc (void)
{
    //Writing 0 to bit 0 selects DC lead-off detect frequency.
    ads119x_write_register (ADS119X_REG_LOFF, DISABLE << 0);
}
void ads119x_lead_off_freq_ac (void)
{
    //Writing 1 to bit selects AC lead-off detect frequency.
    ads119x_write_register (ADS119X_REG_LOFF, ENABLE << 0);
}


//CH1SET register
ads119x_ret_val_t ads119x_get_ch1set_register(uint8_t *reg_ch1set_data)
{
     ads119x_ret_val_t ret_val = ads119x_read_register (ADS119X_REG_CH1SET, reg_ch1set_data);

     return ret_val;
}

void ads119x_ch1_power_down (void)
{
    //Writing 1 to bit 7 puts CH1 into power down.
    ads119x_write_register (ADS119X_REG_CH1SET, ENABLE << 7);
}

void ads119x_ch1_normal_operation (void)
{
    //Writing 0 to bit 7 puts channel 1 into normal operation mode.
    ads119x_write_register (ADS119X_REG_CH1SET, DISABLE << 7);
}

void ads119x_ch1_pga_gain_set_val (ads119x_channel_pga_gain_t pga_gain_val)
{
    //Bit 6 contains the MSB for PGA gain value. To place 3-bit number MSB to
    //bit position 6, logical shift of 4 is needed (2 to 6).
    ads119x_write_register (ADS119X_REG_CH1SET, pga_gain_val << 4);
}

void ads119x_ch1_input_source_select (ads119x_ch_input_source_t ch_input_source)
{
    ads119x_write_register (ADS119X_REG_CH1SET, ch_input_source);
}

//CH2SET register
ads119x_ret_val_t ads119x_get_ch2set_register(uint8_t *reg_ch2set_data)
{
     ads119x_ret_val_t ret_val = ads119x_read_register (ADS119X_REG_CH2SET, reg_ch2set_data);

     return ret_val;
}

void ads119x_ch2_power_down (void)
{
    //Writing 1 to bit 7 puts CH2 into power down.
    ads119x_write_register (ADS119X_REG_CH2SET, ENABLE << 7);
}

void ads119x_ch2_normal_operation (void)
{
    //Writing 0 to bit 7 puts CH2 into normal operation.
    ads119x_write_register (ADS119X_REG_CH2SET, DISABLE << 7);
}

void ads119x_ch2_pga_gain_set_val (ads119x_channel_pga_gain_t pga_gain_val)
{
    //Bit 6 contains the MSB for PGA gain value. To place 3-bit number MSB to
    //bit position 6, logical shift of 4 is needed (2 to 6).
    ads119x_write_register (ADS119X_REG_CH2SET, pga_gain_val << 4);
}

void ads119x_ch2_input_source_select (ads119x_ch_input_source_t ch_input_source)
{
    ads119x_write_register (ADS119X_REG_CH2SET, ch_input_source);
}


//RLD_SENS register
ads119x_ret_val_t ads119x_get_rld_sens_register(uint8_t *reg_rld_sens_data)
{
     ads119x_ret_val_t ret_val = ads119x_read_register(ADS119X_REG_RLD_SENS, reg_rld_sens_data);

      return ret_val;
}

void ads119x_rld_buffer_enable (void)
{
    ads119x_write_register (ADS119X_REG_RLD_SENS, ENABLE << 5);
}

void ads119x_rld_buffer_disable (void)
{
    ads119x_write_register (ADS119X_REG_RLD_SENS, DISABLE << 5);
}

void ads119x_rld_lead_off_sense_enable (void)
{
    ads119x_write_register (ADS119X_REG_RLD_SENS, ENABLE << 4);
}

void ads119x_rld_lead_off_sense_disable (void)
{
    ads119x_write_register (ADS119X_REG_RLD_SENS, DISABLE << 4);
}

void ads119x_rld_ch1_neg_enable (void)
{
    ads119x_write_register (ADS119X_REG_RLD_SENS, ENABLE << 1);
}

void ads118x_rld_ch1_neg_disable (void)
{
    ads119x_write_register (ADS119X_REG_RLD_SENS, DISABLE << 1);
}

void ads119x_rld_ch1_pos_enable (void)
{
    ads119x_write_register (ADS119X_REG_RLD_SENS, ENABLE << 0);
}

void ads119x_rld_ch1_pos_disable (void)
{
    ads119x_write_register (ADS119X_REG_RLD_SENS, DISABLE << 0);
}

void ads119x_rld_ch2_neg_enable (void)
{
    ads119x_write_register (ADS119X_REG_RLD_SENS, ENABLE << 3);
}

void ads118x_rld_ch2_neg_disable (void)
{
    ads119x_write_register (ADS119X_REG_RLD_SENS, DISABLE << 3);
}

void ads119x_rld_ch2_pos_enable (void)
{
    ads119x_write_register (ADS119X_REG_RLD_SENS, ENABLE << 2);
}


void ads119x_rld_ch2_pos_disable (void)
{
    ads119x_write_register (ADS119X_REG_RLD_SENS, DISABLE << 2);
}


//LOFF_SENS register
ads119x_ret_val_t ads119x_get_loff_sens_register(uint8_t *reg_loff_sens_data)
{
     ads119x_ret_val_t ret_val = ads119x_read_register(ADS119X_REG_LOFF_SENS, reg_loff_sens_data);

     return ret_val;
}

void ads119x_lead_off_ch1_curr_dir_enable (void)
{
    ads119x_write_register (ADS119X_REG_LOFF_SENS, ENABLE << 4);
}

void ads119x_lead_off_ch1_curr_dir_disable (void)
{
    ads119x_write_register (ADS119X_REG_LOFF_SENS, DISABLE << 4);
}

void ads119x_lead_off_ch2_curr_dir_enable (void)
{
    ads119x_write_register (ADS119X_REG_LOFF_SENS, ENABLE << 5);
}

void ads119x_lead_off_ch2_curr_dir_disable (void)
{
    ads119x_write_register (ADS119X_REG_LOFF_SENS, DISABLE << 5);
}

void ads119x_lead_off_ch1_pos_in_enable (void)
{
    ads119x_write_register (ADS119X_REG_LOFF_SENS, ENABLE << 0);
}

void ads119x_lead_off_ch1_pos_in_disable (void)
{
    ads119x_write_register (ADS119X_REG_LOFF_SENS, DISABLE << 0);
}

void ads119x_lead_off_ch1_neg_in_enable (void)
{
    ads119x_write_register (ADS119X_REG_LOFF_SENS, ENABLE << 1);
}

void ads119x_lead_off_ch1_neg_in_disable (void)
{
    ads119x_write_register (ADS119X_REG_LOFF_SENS, DISABLE << 1);
}

void ads119x_lead_off_ch2_pos_in_enable (void)
{
    ads119x_write_register (ADS119X_REG_LOFF_SENS, ENABLE << 2);
}

void ads119x_lead_off_ch2_pos_in_disable (void)
{
    ads119x_write_register (ADS119X_REG_LOFF_SENS, DISABLE << 2);
}

void ads119x_lead_off_ch2_neg_in_enable (void)
{
    ads119x_write_register (ADS119X_REG_LOFF_SENS, ENABLE << 3);
}

void ads119x_lead_off_ch2_neg_in_disable (void)
{
    ads119x_write_register (ADS119X_REG_LOFF_SENS, DISABLE << 3);
}

//LOFF_STAT register
ads119x_ret_val_t ads119x_get_loff_state_register(uint8_t *reg_loff_state_data)
{
     ads119x_ret_val_t ret_val = ads119x_read_register (ADS119X_REG_LOFF_STAT, reg_loff_state_data);

     return ret_val;
}

void ads119x_set_clock_div_4 (void)
{
    ads119x_write_register (ADS119X_REG_LOFF_STAT, DISABLE << 6);
}

void ads119x_set_clock_div_16 (void)
{
    ads119x_write_register (ADS119X_REG_LOFF_STAT, ENABLE << 6);
}

bool ads119x_get_rld_connect_status (uint8_t reg_loff_stat_data)
{
    bool rld_connect_status = false;

    //If bit 4 is set, RLD is NOT connected.
    if (reg_loff_stat_data & (ENABLE << 4))
    {
        rld_connect_status = false;
    }
    else
    {
        rld_connect_status = true;
    }

    return rld_connect_status;
}

bool ads119x_get_ch1_neg_connect_status (uint8_t reg_loff_stat_data)
{
    bool ch1_neg_connect_status = false;

    //If bit 4 is set, RLD is NOT connected.
    if (reg_loff_stat_data & (ENABLE << 1))
    {
        ch1_neg_connect_status = false;
    }
    else
    {
        ch1_neg_connect_status = true;
    }

    return ch1_neg_connect_status;
}

bool ads119x_get_ch1_pos_connect_status (uint8_t reg_loff_stat_data)
{
    bool ch1_pos_connect_status = false;

    //If bit 4 is set, RLD is NOT connected.
    if (reg_loff_stat_data & (ENABLE << 0))
    {
        ch1_pos_connect_status = false;
    }
    else
    {
        ch1_pos_connect_status = true;
    }

    return ch1_pos_connect_status;
}

bool ads119x_get_ch2_neg_connect_status (uint8_t reg_loff_stat_data)
{
    bool ch2_neg_connect_status = false;

    //If bit 4 is set, RLD is NOT connected.
    if (reg_loff_stat_data & (ENABLE << 3))
    {
        ch2_neg_connect_status = false;
    }
    else
    {
        ch2_neg_connect_status = true;
    }

    return ch2_neg_connect_status;
}

bool ads19xx_get_ch2_pos_connect_status (uint8_t reg_loff_stat_data)
{
    bool ch2_pos_connect_status = false;

    //If bit 4 is set, RLD is NOT connected.
    if (reg_loff_stat_data & (ENABLE << 2))
    {
        ch2_pos_connect_status = false;
    }
    else
    {
        ch2_pos_connect_status = true;
    }

    return ch2_pos_connect_status;
}

//MISC1 register
ads119x_ret_val_t ads119x_get_misc1_register(uint8_t *reg_misc1_data)
{
     ads119x_ret_val_t ret_val = ads119x_read_register (ADS119X_REG_MISC1, reg_misc1_data);

     return ret_val;
}


//MISC2 register
ads119x_ret_val_t ads119x_get_misc2_register(uint8_t *reg_misc2_data)
{
     ads119x_ret_val_t ret_val = ads119x_read_register (ADS119X_REG_MISC2, reg_misc2_data);

     return ret_val;
}

void ads119x_offset_calibration_enable (void)
{
    ads119x_write_register (ADS119X_REG_MISC2, ENABLE << 7); 
}

void ads119x_offset_calibration_disable (void)
{
    ads119x_write_register (ADS119X_REG_MISC2, DISABLE << 7);
}

void ads119x_rld_reference_set_external (void)
{
    ads119x_write_register (ADS119X_REG_MISC2, DISABLE << 0);
}

void ads119x_rld_reference_set_internal (void)
{
    ads119x_write_register (ADS119X_REG_MISC2, ENABLE << 1);
}

//GPIO register
ads119x_ret_val_t ads119x_get_gpio_register (uint8_t * reg_gpio_data)
{
     ads119x_ret_val_t ret_val = ads119x_read_register (ADS119X_REG_GPIO, reg_gpio_data);

     return ret_val;
}

void ads119x_gpio1_set_as_input (void)
{
    ads119x_write_register (ADS119X_REG_GPIO, ENABLE << 2);
}
void ads119x_gpio1_set_as_output (void)
{
    ads119x_write_register (ADS119X_REG_GPIO, DISABLE << 2);
}

void ads119x_gpio1_output_set (void)
{
    ads119x_write_register (ADS119X_REG_GPIO, ENABLE << 0);
}

void ads119x_gpio1_output_reset (void)
{
    ads119x_write_register (ADS119X_REG_GPIO, DISABLE << 0);
}

bool ads119x_gpio1_status_get (uint8_t reg_gpio_data)
{
    bool gpio1_status = reg_gpio_data & (ENABLE << 0);

    return gpio1_status;
}

void ads119x_gpio2_set_as_input (void)
{
    ads119x_write_register (ADS119X_REG_GPIO, ENABLE << 3);
}

void ads119x_gpio2_set_as_output (void)
{
    ads119x_write_register (ADS119X_REG_GPIO, DISABLE << 3);
}

void ads119x_gpio2_output_set (void)
{
    ads119x_write_register (ADS119X_REG_GPIO, ENABLE << 1);
}

void ads119x_gpio2_output_reset (void)
{
    ads119x_write_register (ADS119X_REG_GPIO, DISABLE << 1);
}

bool ads119x_gpio2_status_get (uint8_t reg_gpio_data)
{
    bool gpio2_status = reg_gpio_data & (ENABLE << 1);

    return gpio2_status;
}