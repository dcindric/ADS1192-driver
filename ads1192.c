/**
 * @file ads1192.c
 * @author Dino Cindric
 * @brief 
 * @version 0.1
 * @date 2023-01-22
 * 
 */

#include "ads1192.h"

// Global ADS119X configuration structure, accessed by most of the driver functions.
ads119x_config_t ads119x_config = {0};


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


void ads119x_set_data_rate(ads119x_data_rate_t ads119x_data_rate)
{
    ads119x_write_register (ADS119X_REG_CONFIG1, (uint8_t) ads119x_data_rate);
}

ads119x_data_rate_t ads119x_get_data_rate(const uint8_t reg_config1_data)
{
    //500 SPS is the default value.
    ads119x_data_rate_t data_rate = reg_config1_data & 0x07;
    return data_rate;
}


//CONFIG2 register
ads119x_ret_val_t ads119x_get_config_2_register(uint8_t *reg_config2_data)
{
    ads119x_ret_val_t ret_val = ads119x_read_register(ADS119X_REG_CONFIG2, reg_config2_data);
    return ret_val;
}

void ads119x_control_lead_off_comparator (enable_t command)
{
    ads119x_write_register (ADS119X_REG_CONFIG2, command << 6);
}

bool ads119x_get_lead_off_comparator_status (const uint8_t reg_config_2_data)
{
    bool lead_off_comp_status = reg_config_2_data & (ENABLE << 6);
    return lead_off_comp_status;
}


void ads119x_control_reference_buffer (enable_t command)
{
    ads119x_write_register (ADS119X_REG_CONFIG2, command << 5);
}

bool ads119x_get_reference_buffer_status (const uint8_t reg_config_2_data)
{
    bool reference_buff_status = reg_config_2_data & (ENABLE << 5);
    return reference_buff_status;
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

ads119x_reference_t ads119x_get_volt_reference (const uint8_t reg_config_2_data)
{
    ads119x_reference_t volt_reference = 0;

    if (reg_config_2_data & (ENABLE << 4))
    {
        volt_reference = ADS119X_4_033V;
    }
    else
    {
        volt_reference = ADS119X_2_42V;
    }

    return volt_reference;
}


void ads119x_control_clock_osc_out (enable_t command)
{
    ads119x_write_register (ADS119X_REG_CONFIG2, command << 3);
}


bool ads119x_get_clock_osc_out_status (const uint8_t reg_config_2_data)
{
    bool clock_osc_out_status = reg_config_2_data & (ENABLE << 3);
    return clock_osc_out_status;
}

void ads119x_control_test_signal (enable_t command)
{
    //Writing 1 to bit 1 enables the test signal.
    ads119x_write_register (ADS119X_REG_CONFIG2, command << 1);
}

bool ads119x_get_test_signal_status (const uint8_t reg_config_2_data)
{
    bool test_signal_status = reg_config_2_data & (ENABLE << 1);
    return test_signal_status;
}


void ads119x_set_test_signalfreq_dc (void)
{
    //Writing 0 to bit 0 sets test signal frequency to 0 Hz (dc).
    ads119x_write_register (ADS119X_REG_CONFIG2, DISABLE << 0);
}

void ads119x_set_test_signalfreq_1hz (void)
{
    //Writing 1 to bit 0 sets test signal frequency to square wave at 1 Hz.
    ads119x_write_register (ADS119X_REG_CONFIG2, ENABLE << 0);
}

bool ads119x_get_test_signalfreq_status (const uint8_t reg_config_2_data)
{
    bool test_signal_freq = reg_config_2_data & (ENABLE << 0);
    return test_signal_freq;
}


//LOFF register
ads119x_ret_val_t ads119x_get_loff_register(uint8_t *reg_loff_data)
{
     ads119x_ret_val_t ret_val = ads119x_read_register(ADS119X_REG_LOFF, reg_loff_data);
     return ret_val;
}


void ads119x_set_lead_off_comparator_thres (ads119x_lead_off_comp_pos_side_val_t comp_pos_val)
{
    uint8_t reg_write_val = (uint8_t) comp_pos_val << 5;

    ads119x_write_register (ADS119X_REG_LOFF, reg_write_val);  
}

ads119x_lead_off_comp_pos_side_val_t ads119x_get_lead_off_comparator_pos_thres (const uint8_t reg_loff_data)
{
    //Lead-off comparator positive side information is contained in 3 MSBs.
    ads119x_lead_off_comp_pos_side_val_t lead_off_comp_pos_val = (reg_loff_data & 0xE0) >> 5;
    return lead_off_comp_pos_val;
}

ads119x_lead_off_comp_neg_side_val_t ads119x_get_lead_off_comparator_neg_thres (const uint8_t reg_loff_data)
{
    //Lead-off comparator negative side information is contained in 3 MSBs.
    ads119x_lead_off_comp_pos_side_val_t lead_off_comp_pos_val = (reg_loff_data & 0xE0) >> 5;
    return lead_off_comp_pos_val;
}

void ads119x_set_lead_off_current_val (ads119x_lead_off_curr_val_t lead_off_curr_val)
{
    uint8_t reg_write_val = (uint8_t) lead_off_curr_val << 2;
    ads119x_write_register (ADS119X_REG_LOFF, lead_off_curr_val);
}

ads119x_lead_off_curr_val_t ads119x_get_lead_off_current_val (const uint8_t reg_loff_data)
{
    ads119x_lead_off_curr_val_t lead_off_curr_val = (reg_loff_data & 0x0C) >> 2;
    return lead_off_curr_val;
}

void ads119x_set_lead_off_freq_dc (void)
{
    //Writing 0 to bit 0 selects DC lead-off detect frequency.
    ads119x_write_register (ADS119X_REG_LOFF, DISABLE << 0);
}
void ads119x_set_lead_off_freq_ac (void)
{
    //Writing 1 to bit selects AC lead-off detect frequency.
    ads119x_write_register (ADS119X_REG_LOFF, ENABLE << 0);
}

bool ads119x_get_lead_off_freq_status (const uint8_t reg_loff_data)
{
    bool lead_off_freq_status = reg_loff_data & (ENABLE << 0);
    return lead_off_freq_status;
}

//CH1SET register
ads119x_ret_val_t ads119x_get_ch1set_register(uint8_t *reg_ch1set_data)
{
     ads119x_ret_val_t ret_val = ads119x_read_register (ADS119X_REG_CH1SET, reg_ch1set_data);
     return ret_val;
}

void ads119x_set_ch1_power_down (void)
{
    //Writing 1 to bit 7 puts CH1 into power down.
    ads119x_write_register (ADS119X_REG_CH1SET, ENABLE << 7);
}

void ads119x_set_ch1_normal_operation (void)
{
    //Writing 0 to bit 7 puts channel 1 into normal operation mode.
    ads119x_write_register (ADS119X_REG_CH1SET, DISABLE << 7);
}

bool ads119x_get_ch1_power_status (const uint8_t reg_ch1set_data)
{
    bool ch1_power_status = reg_ch1set_data >> 7;
    return ch1_power_status;
}

void ads119x_set_ch1_pga_gain_val (ads119x_channel_pga_gain_t pga_gain_val)
{
    //Bit 6 contains the MSB for PGA gain value. To place 3-bit number MSB to
    //bit position 6, logical shift of 4 is needed (2 to 6).
    ads119x_write_register (ADS119X_REG_CH1SET, pga_gain_val << 4);
}

ads119x_channel_pga_gain_t ads119x_get_ch1_pga_gain_val (const uint8_t reg_ch1set_data)
{
    ads119x_channel_pga_gain_t ch1_pga_gain = (reg_ch1set_data & 0x70) >> 4;
    return ch1_pga_gain;
}

void ads119x_set_ch1_input_source (ads119x_ch_input_source_t ch_input_source)
{
    ads119x_write_register (ADS119X_REG_CH1SET, ch_input_source);
}

ads119x_ch_input_source_t ads119x_get_ch1_input_source (const uint8_t reg_ch1set_data)
{
    ads119x_ch_input_source_t ch1_input_source = reg_ch1set_data & 0x0F;
    return ch1_input_source;
}

//CH2SET register
ads119x_ret_val_t ads119x_get_ch2set_register(uint8_t *reg_ch2set_data)
{
     ads119x_ret_val_t ret_val = ads119x_read_register (ADS119X_REG_CH2SET, reg_ch2set_data);
     return ret_val;
}

void ads119x_set_ch2_power_down (void)
{
    //Writing 1 to bit 7 puts CH2 into power down.
    ads119x_write_register (ADS119X_REG_CH2SET, ENABLE << 7);
}

void ads119x_set_ch2_normal_operation (void)
{
    //Writing 0 to bit 7 puts CH2 into normal operation.
    ads119x_write_register (ADS119X_REG_CH2SET, DISABLE << 7);
}

bool ads119x_get_ch2_power_status (const uint8_t reg_ch2set_data)
{
    bool ch2_power_status = reg_ch2set_data >> 7;
    return ch2_power_status;
}


void ads119x_set_ch2_pga_gain_val (ads119x_channel_pga_gain_t pga_gain_val)
{
    //Bit 6 contains the MSB for PGA gain value. To place 3-bit number MSB to
    //bit position 6, logical shift of 4 is needed (2 to 6).
    ads119x_write_register (ADS119X_REG_CH2SET, pga_gain_val << 4);
}

ads119x_channel_pga_gain_t ads119x_get_ch2_pga_gain_val (const uint8_t reg_ch2set_data)
{
    ads119x_channel_pga_gain_t ch2_pga_gain_val = (reg_ch2set_data & 0x70) >> 4;
    return ch2_pga_gain_val;
}

void ads119x_set_ch2_input_source (ads119x_ch_input_source_t ch_input_source)
{
    ads119x_write_register (ADS119X_REG_CH2SET, ch_input_source);
}

ads119x_ch_input_source_t ads119x_get_ch2_input_source (const uint8_t reg_ch2set_data)
{
    ads119x_ch_input_source_t ch2_input_source = reg_ch2set_data & 0x0F;
    return ch2_input_source;
}


//RLD_SENS register
ads119x_ret_val_t ads119x_get_rld_sens_register(uint8_t *reg_rld_sens_data)
{
    ads119x_ret_val_t ret_val = ads119x_read_register(ADS119X_REG_RLD_SENS, reg_rld_sens_data);
    return ret_val;
}

void ads119x_enable_rld_buffer (void)
{
    ads119x_write_register (ADS119X_REG_RLD_SENS, ENABLE << 5);
}

void ads119x_disable_rld_buffer (void)
{
    ads119x_write_register (ADS119X_REG_RLD_SENS, DISABLE << 5);
}

bool ads119x_get_rld_buffer_status (const reg_rld_sens_data)
{
    bool rld_buffer_status = reg_rld_sens_data & (ENABLE << 5);
    return rld_buffer_status;
}


void ads119x_enable_rld_lead_off_sense (void)
{
    ads119x_write_register (ADS119X_REG_RLD_SENS, ENABLE << 4);
}

void ads119x_disable_rld_lead_off_sense (void)
{
    ads119x_write_register (ADS119X_REG_RLD_SENS, DISABLE << 4);
}

bool ads119x_get_rld_lead_off_sense_status (const reg_rld_sens_data)
{
    bool rld_lead_off_sense_status = reg_rld_sens_data & (ENABLE << 4);
    return rld_lead_off_sense_status;
}


void ads119x_enable_rld_ch1_neg (void)
{
    ads119x_write_register (ADS119X_REG_RLD_SENS, ENABLE << 1);
}

void ads118x_disable_rld_ch1_neg (void)
{
    ads119x_write_register (ADS119X_REG_RLD_SENS, DISABLE << 1);
}

bool ads119x_get_rld_ch1_neg_status (const reg_rld_sens_data)
{
    bool rld_ch1_neg_status = reg_rld_sens_data & (ENABLE << 1);
    return rld_ch1_neg_status;
}

void ads119x_enable_rld_ch1_pos (void)
{
    ads119x_write_register (ADS119X_REG_RLD_SENS, ENABLE << 0);
}

void ads119x_disable_rld_ch1_pos (void)
{
    ads119x_write_register (ADS119X_REG_RLD_SENS, DISABLE << 0);
}

bool ads119x_get_rld_ch1_pos_status (const reg_rld_sens_data)
{
    bool rld_ch1_pos_status = reg_rld_sens_data & (ENABLE << 0);
    return rld_ch1_pos_status;
}

void ads119x_enable_rld_ch2_neg (void)
{
    ads119x_write_register (ADS119X_REG_RLD_SENS, ENABLE << 3);
}

void ads118x_disable_rld_ch2_neg (void)
{
    ads119x_write_register (ADS119X_REG_RLD_SENS, DISABLE << 3);
}

bool ads119x_get_rld_ch2_neg_status (const reg_rld_sens_data)
{
    bool rld_ch2_neg_status = reg_rld_sens_data & (ENABLE << 3);
    return rld_ch2_neg_status;
}

void ads119x_enable_rld_ch2_pos (void)
{
    ads119x_write_register (ADS119X_REG_RLD_SENS, ENABLE << 2);
}

void ads119x_disable_rld_ch2_pos (void)
{
    ads119x_write_register (ADS119X_REG_RLD_SENS, DISABLE << 2);
}

bool ads119x_get_rld_ch2_pos_status (const reg_rld_sens_data)
{
    bool rld_ch2_pos_status = reg_rld_sens_data & (ENABLe << 2);

    return rld_ch2_pos_status;
}


//LOFF_SENS register
ads119x_ret_val_t ads119x_get_loff_sens_register(uint8_t *reg_loff_sens_data)
{
     ads119x_ret_val_t ret_val = ads119x_read_register(ADS119X_REG_LOFF_SENS, reg_loff_sens_data);

     return ret_val;
}

void ads119x_control_lead_off_curr_dir (ads119x_ch_idx_t ch_idx, uint8_t command)
{
    uint8_t reg_write_val = 0;

    if (ADS119X_CH1 == ch_idx)
    {
        reg_write_val = command << 4;
    }

    else if (ADS119X_CH2 == ch_idx)
    {
        reg_write_val = command << 5;
    }

    ads119x_write_register (ADS119X_REG_LOFF_SENS, reg_write_val);
}

bool ads119x_get_lead_off_curr_dir_status (const uint8_t reg_loff_sens_data, ads119x_ch_idx_t ch_idx)
{
    bool lead_off_curr_dir_status = 0;

    if (ADS119X_CH1 == ch_idx)
    {
        lead_off_curr_dir_status = reg_loff_sens_data & (ENABLE << 4);
    }

    else if (ADS119X_CH2 == ch_idx)
    {
        lead_off_curr_dir_status = reg_loff_sens_data & (ENABLE << 5);
    }

    return lead_off_curr_dir_status;
}


void ads119x_control_lead_off_pos_in (ads119x_ch_idx_t ch_idx, uint8_t command)
{
    uint8_t reg_write_val = false;

    if (ADS119X_CH1 == ch_idx)
    {
        reg_write_val = command << 0;
    }
    else if (ADS119X_CH2 == ch_idx)
    {
        reg_write_val = command << 2;
    }

    ads119x_write_register (ADS119X_REG_LOFF_SENS, reg_write_val);
}

bool ads119x_get_lead_off_pos_in_status (const uint8_t reg_loff_sens_data, ads119x_ch_idx_t ch_idx)
{
    bool lead_off_pos_in_status = false;

    if (ADS119X_CH1 == ch_idx)
    {
        lead_off_pos_in_status = reg_loff_sens_data & (ENABLE << 0);
    }
    else if (ADS119X_CH2 == ch_idx)
    {
        lead_off_pos_in_status = reg_loff_sens_data & (ENABLE << 2);
    }

    return lead_off_pos_in_status;
}

void ads119x_control_lead_off_neg_in (ads119x_ch_idx_t ch_idx, uint8_t command)
{
    uint8_t reg_write_val = 0;

    if (ADS119X_CH1 == ch_idx)
    {
        reg_write_val = command << 1;
    }
    else if (ADS119X_CH2 == ch_idx)
    {
        reg_write_val = command << 3;
    }

    ads119x_write_register (ADS119X_REG_LOFF_SENS, reg_write_val);
}

bool ads119x_get_lead_off_neg_in_status (const uint8_t reg_loff_sens_data, ads119x_ch_idx_t ch_idx)
{
    bool lead_off_neg_in_status = false;

    if (ADS119X_CH1 == reg_loff_sens_data)
    {
        lead_off_neg_in_status = reg_loff_sens_data & (ENABLE << 1);
    }
    else if (ADS119X_CH2 == reg_loff_sens_data)
    {
        lead_off_neg_in_status = reg_loff_sens_data & (ENABLE << 3);
    }

    return lead_off_neg_in_status;
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
    //If bit 4 is set, RLD is NOT connected.
    bool rld_connect_status = reg_loff_stat_data & (ENABLE << 4);
    return rld_connect_status;
}

bool ads119x_get_pos_elect_connect_status (const uint8_t reg_loff_stat_data, ads119x_ch_idx_t ch_idx)
{
    bool pos_elect_connect_status = false; 

    if (ADS119X_CH1 == ch_idx)
    {
        pos_elect_connect_status = reg_loff_stat_data & (ENABLE << 0);
    }
    else if (ADS119X_CH2 == ch_idx)
    {
        pos_elect_connect_status = reg_loff_stat_data & (ENABLE << 2);
    }

    return pos_elect_connect_status;
}

bool ads119x_get_neg_elect_connect_status (const uint8_t reg_loff_stat_data, ads119x_ch_idx_t ch_idx)
{
    bool neg_elect_connect_status = false;

    if (ADS119X_CH1 == ch_idx)
    {
        neg_elect_connect_status = reg_loff_stat_data & (ENABLE << 1);
    }
    else if (ADS119X_CH2 == ch_idx)
    {
        neg_elect_connect_status = reg_loff_stat_data & (ENABLE << 3);
    }

    return neg_elect_connect_status;
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

void ads119x_control_offset_calibration (uint8_t command)
{
    ads119x_write_register (ADS119X_REG_MISC2, command << 7); 
}

bool ads119x_get_offset_calibration_status (const uint8_t reg_misc2_data)
{
    bool offset_cal_status = reg_misc2_data & (ENABLE << 7);
    return offset_cal_status;
}

void ads119x_set_rld_reference_external (void)
{
    ads119x_write_register (ADS119X_REG_MISC2, DISABLE << 1);
}

void ads119x_set_rld_reference_internal (void)
{
    ads119x_write_register (ADS119X_REG_MISC2, ENABLE << 1);
}

bool ads119x_get_rld_reference_status (const uint8_t reg_misc2_data)
{
    bool offset_cal_status = reg_misc2_data & (ENABLE << 1);
    return offset_cal_status;
}

//GPIO register
ads119x_ret_val_t ads119x_get_gpio_register (uint8_t * reg_gpio_data)
{
     ads119x_ret_val_t ret_val = ads119x_read_register (ADS119X_REG_GPIO, reg_gpio_data);

     return ret_val;
}

void ads119x_set_gpio1_as_input (void)
{
    ads119x_write_register (ADS119X_REG_GPIO, ENABLE << 2);
}
void ads119x_set_gpio1_as_output (void)
{
    ads119x_write_register (ADS119X_REG_GPIO, DISABLE << 2);
}

void ads119x_set_gpio1_output (void)
{
    ads119x_write_register (ADS119X_REG_GPIO, ENABLE << 0);
}

void ads119x_reset_gpio1_output (void)
{
    ads119x_write_register (ADS119X_REG_GPIO, DISABLE << 0);
}

bool ads119x_get_gpio1_status (uint8_t reg_gpio_data)
{
    bool gpio1_status = reg_gpio_data & (ENABLE << 0);

    return gpio1_status;
}

void ads119x_set_gpio2_as_input (void)
{
    ads119x_write_register (ADS119X_REG_GPIO, ENABLE << 3);
}

void ads119x_set_gpio2_as_output (void)
{
    ads119x_write_register (ADS119X_REG_GPIO, DISABLE << 3);
}

void ads119x_set_gpio2_output (void)
{
    ads119x_write_register (ADS119X_REG_GPIO, ENABLE << 1);
}

void ads119x_reset_gpio2_output (void)
{
    ads119x_write_register (ADS119X_REG_GPIO, DISABLE << 1);
}

bool ads119x_gpio2_status_get (uint8_t reg_gpio_data)
{
    bool gpio2_status = reg_gpio_data & (ENABLE << 1);

    return gpio2_status;
}