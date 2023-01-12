#include "ads1192.h"

//Global ADS119X configuration structure, accessed by most of the driver functions.
ads119x_config_t ads119x_config = {0};

/*Function for reading register values in the */
ads119x_ret_val_t ads119x_read_register (uint8_t reg_addr, uint8_t * reg_data)
{
    uint8_t reg_read_opcode[ADS119X_READ_OPCODE_LEN] = {0};

    reg_read_opcode[0] = ADS119X_READ_FIRST_OPCODE_BYTE | reg_addr;
    reg_read_opcode[1] = ADS119X_READ_SECOND_OPCODE_BYTE;


    ads119x_config.f_dev_spi_write (reg_read_opcode, ADS119X_READ_OPCODE_LEN);
    ads119x_config.f_dev_spi_read (reg_data, 1);
}


ads119x_ret_val_t ads119x_write_register (uint8_t reg_addr, uint8_t reg_command)
{
    uint8_t reg_write_data[ADS119X_WRITE_OPCODE_LEN + 1] = {0};

    reg_write_data[0] = ADS119X_WRITE_FIRST_OPCODE_BYTE | reg_addr;
    reg_write_data[1] = ADS119X_WRITE_SECOND_OPCODE_BYTE;
    reg_write_data[2] = reg_command;

    ads119x_config.f_dev_spi_write (reg_write_data, sizeof (reg_write_data));
}


ads119x_ret_val_t ads119x_write_command (ads119x_command_t ads119x_command)
{
    //Command size will always be equal to one byte.
    ads119x_config.f_dev_spi_write (ads119x_command, sizeof (ads119x_command));
}


ads119x_ret_val_t ads119x_get_id_register (uint8_t * reg_id_data)
{
    ads119x_read_register (ADS119X_REG_ID, reg_id_data);
}

ads119x_ret_val_t ads119x_get_config_1_register (uint8_t * reg_config1_data)
{
    ads119x_read_register (ADS119X_REG_CONFIG1, reg_config1_data);
}

ads119x_ret_val_t ads119x_get_config_2_register (uint8_t * reg_config2_data)
{
    ads119x_read_register (ADS119X_REG_CONFIG2, reg_config2_data);
}

ads119x_ret_val_t ads119x_get_loff_register (uint8_t * reg_loff_data)
{
    ads119x_read_register (ADS119X_REG_LOFF, reg_loff_data);
}

ads119x_ret_val_t ads119x_get_ch1set_register (uint8_t * reg_ch1set_data)
{
    ads119x_read_register (ADS119X_REG_CH1SET, reg_ch1set_data);
}

ads119x_ret_val_t ads119x_get_ch2set_register (uint8_t * reg_ch2set_data)
{
    ads119x_read_register (ADS119X_REG_CH2SET, reg_ch2set_data);
}

ads119x_ret_val_t ads119x_get_rld_sens_register (uint8_t * reg_rld_sens_data)
{
    ads119x_read_register (ADS119X_REG_RLD_SENS, reg_rld_sens_data);
}

ads119x_ret_val_t ads119x_get_loff_sens_register (uint8_t * reg_loff_sens_data)
{
    ads119x_read_register (ADS119X_REG_LOFF_SENS, reg_loff_sens_data);
}

ads119x_ret_val_t ads119x_get_loff_state_register (uint8_t * reg_loff_state_data)
{
    ads119x_read_register (ADS119X_REG_LOFF_STAT, reg_loff_state_data);
}

ads119x_ret_val_t ads119x_get_misc1_register (uint8_t * reg_misc1_data)
{
    ads119x_read_register (ADS119X_REG_MISC1, reg_misc1_data);
}

ads119x_ret_val_t ads119x_get_misc2_register (uint8_t * reg_misc2_data)
{
    ads119x_read_register (ADS119X_REG_MISC2, reg_misc2_data);
}