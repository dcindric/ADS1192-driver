#include <stdint.h>
#include <stdbool.h>


#ifndef ENABLE
#define ENABLE 1
#endif

#ifndef DISABLE
#define DISABLE 0
#endif

/*Read opcode bytes definitions*/
#define ADS119X_READ_FIRST_OPCODE_BYTE   (0x20)  
#define ADS119X_READ_SECOND_OPCODE_BYTE  (0x00)  //Number of regs to read - 1.
#define ADS119X_READ_OPCODE_LEN          (2)


#define ADS119X_WRITE_FIRST_OPCODE_BYTE (0x40)
#define ADS119X_WRITE_SECOND_OPCODE_BYTE (0x00)
#define ADS119X_WRITE_OPCODE_LEN        (2)


/*Register address definitions*/

/*Device settings (read-only registers)*/
#define ADS119X_REG_ID      (0x00)

/*Global settings accross channels*/
#define ADS119X_REG_CONFIG1    (0x01)   //Configures each ADC channel sample rate
#define ADS119X_REG_CONFIG2    (0x02)   
#define ADS119X_REG_LOFF       (0x03)

/*Channel specific settings*/
#define ADS119X_REG_CH1SET     (0x04)
#define ADS119X_REG_CH2SET     (0x05)
#define ADS119X_REG_RLD_SENS    (0x06)
#define ADS119X_REG_LOFF_SENS   (0x07)
#define ADS119X_REG_LOFF_STAT   (0x08)


/*GPIO and other registers*/
#define ADS119X_REG_MISC1       (0x09)
#define ADS119X_REG_MISC2       (0x0A)
#define ADS119X_REG_GPIO        (0x0B)

typedef enum
{
    //System commands
    ADS119X_WAKEUP = 0x02,
    ADS119X_STANDBY = 0x04,
    ADS119X_RESET = 0x06,
    ADS119X_START = 0x08,
    ADS119X_STOP = 0x0A,
    ADS119X_OFFSETCAL = 0x1A,

    //Data read commands
    ADS119X_RDATAC = 0x10,
    ADS119X_SDATAC = 0x11,
    ADS119X_RDATA = 0x12

} ads119x_command_t;
typedef enum
{
    ADS119X_RET_SUCCESS = 0,
    ADS119X_RET_FAIL    

} ads119x_ret_val_t;


typedef enum
{
    ADS119X_DATA_RATE_125_SPS = 0,
    ADS119X_DATA_RATE_250_SPS = 1,
    ADS119X_DATA_RATE_500_SPS = 2,
    ADS119X_DATA_RATE_1000_SPS = 3,
    ADS119X_DATA_RATE_2000_SPS = 4,
    ADS119X_DATA_RATE_4000_SPS = 5,
    ADS119X_DATA_RATE_8000_SPS = 6,
    ADS119X_DATA_RATE_INVALID = 7

} ads119x_data_rate_t;

typedef enum
{
    ADS1191 = 0,
    ADS1192 = 1,
    ADS1291 = 2,
    ADS1292 = 3

} ads119x_device_id_t;

typedef enum
{
    ADS119X_CONV_MODE_CONT = 0,
    ADS119X_CONV_MODE_SINGLE

} asd119x_conversion_mode_t;

typedef enum
{
    ADS119X_2_42V = 0,
    ADS119X_4_033V = 1

} ads119x_reference_t;


typedef enum
{
    ADS119X_COMP_POS_95 = 0,
    ADS119X_COMP_POS_92_5,
    ADS119X_COMP_POS_90,
    ADS119X_COMP_POS_87_5,
    ADS119X_COMP_POS_85,
    ADS119X_COMP_POS_80,
    ADS119X_COMP_POS_75,
    ADS119X_COMP_POS_70
    
} ads119x_lead_off_comp_pos_side_val_t;

typedef enum
{
    ADS119X_COMP_NEG_5 = 0,
    ADS119X_COMP_NEG_7_5,
    ADS119X_COMP_NEG_10,
    ADS119X_COMP_NEG_12_5,
    ADS119X_COMP_NEG_15,
    ADS119X_COMP_NEG_20,
    ADS119X_COMP_NEG_25,
    ADS119X_COMP_NEG_30

} ads119x_lead_off_comp_neg_side_val_t;

typedef enum
{
    ADS119X_LEAD_OFF_CURR_6_NA = 0,
    ADS119X_LEAD_OFF_CURR_22_NA,
    ADS119X_LEAD_OFF_CURR_6_UA,
    ADS119X_LEAD_OFF_CURR_22UA

} ads119x_lead_off_curr_val_t;


typedef enum
{
    ADS119X_PGA_GAIN_6 = 0, //Default PGA gain value is 6.
    ADS119X_PGA_GAIN_1,
    ADS119X_PGA_GAIN_2,
    ADS119X_PGA_GAIN_3,
    ADS119X_PGA_GAIN_4,
    ADS119X_PGA_GAIN_8,
    ADS119X_PGA_GAIN_12

} ads119x_channel_pga_gain_t;


typedef enum
{
    ADS119X_NORMAL_ELEC_IN = 0,
    ADS119X_INPUT_SHORTED,
    ADS119X_RLD_MEASURE,
    ADS119X_MVDD_SUPPLY_MEASURE,
    ADS119X_TEMP_SENSOR,
    ADS119X_TEST_SIGNAL,
    ADS119X_RLD_DRP,
    ADS119X_RLD_DRM,
    ADS119X_RLD_DRPM,
    ADS119X_IN3P_IN3N

} ads119x_ch_input_source_t;


/*Function pointer type declarations*/
typedef ads119x_ret_val_t (*ads119x_spi_write) (uint8_t * send_buffer, uint8_t len);
typedef ads119x_ret_val_t (*ads119x_spi_read) (uint8_t * recv_buffer, uint8_t len);
typedef ads119x_ret_val_t (*ads119x_start_set) (uint8_t start_pin_state);
typedef ads119x_ret_val_t (*ads119x_reset) (uint8_t reset_pin_state);

typedef struct
{
    ads119x_spi_read f_dev_spi_read; 
    ads119x_spi_write f_dev_spi_write;
    ads119x_start_set f_dev_start_set;
    ads119x_reset f_dev_reset;

} ads119x_config_t;

typedef struct
{
    uint16_t ch1_raw_data;
    uint16_t ch2_raw_data;
    uint16_t status_info;

} ads119x_meas_t;


ads119x_ret_val_t ads119x_read_register (const uint8_t reg_addr, uint8_t * reg_data);
ads119x_ret_val_t ads119x_write_register (uint8_t reg_addr, uint8_t reg_data);

ads119x_ret_val_t ads119x_write_command (ads119x_command_t ads119x_command);

//ID register
ads119x_ret_val_t ads119x_get_id_register (uint8_t * reg_id_data);
void ads119x_get_device_id (ads119x_device_id_t * ads119x_device_id);

//CONFIG1 register
asd119x_conversion_mode_t ads119x_get_conversion_mode (const uint8_t reg_config1_data);
void ads119x_set_conversion_mode (asd119x_conversion_mode_t conv_mode);

ads119x_data_rate_t ads119x_get_data_rate(const uint8_t reg_config1_data);
void ads119x_set_data_rate (ads119x_data_rate_t ads119x_data_rate);


//CONFIG2 register
ads119x_ret_val_t ads119x_get_config_2_register (uint8_t * reg_config_2_data);

void ads119x_enable_lead_off_comparator (void);
void ads119x_disable_lead_off_comparator (void);

void ads119x_enable_reference_buffer (void);
void ads119x_disable_reference_buffer (void);

void ads119x_set_volt_reference (ads119x_reference_t volt_reference_val);

void ads119x_enable_clock_osc_out (void);
void ads119x_disable_clock_osc_out (void);

void ads119x_enable_test_signal (void);
void ads119x_disable_test_signal (void);
void ads119x_set_test_signalfreq_dc (void);
void ads119x_set_test_signalfreq_1hz (void);

//LOFF register
ads119x_ret_val_t ads119x_get_loff_register (uint8_t * reg_loff_data);

void ads119x_set_lead_off_comparatorpos_val (ads119x_lead_off_comp_pos_side_val_t comp_pos_val);
void ads119x_set_lead_off_comparator_neg_val (ads119x_lead_off_comp_neg_side_val_t comp_neg_val);

void ads119x_set_lead_off_current_val (ads119x_lead_off_curr_val_t lead_off_curr_val);
void ads119x_set_lead_off_freq_dc (void);
void ads119x_set_lead_off_freq_ac (void);

//CH1SET register
ads119x_ret_val_t ads119x_get_ch1set_register (uint8_t * reg_ch1set_data);

void ads119x_set_ch1_power_down (void);
void ads119x_set_ch1_normal_operation (void);
void ads119x_set_ch1_pga_gain_val (ads119x_channel_pga_gain_t pga_gain_val);
void ads119x_set_ch1_input_source (ads119x_ch_input_source_t ch_input_source);

//CH2 register
ads119x_ret_val_t ads119x_get_ch2set_register (uint8_t * reg_ch2set_data);

void ads119x_set_ch2_power_down (void);
void ads119x_set_ch2_normal_operation (void);
void ads119x_set_ch2_pga_gain_val (ads119x_channel_pga_gain_t pga_gain_val);
void ads119x_set_ch2_input_source (ads119x_ch_input_source_t ch_input_source);

//RLD_SENS register
ads119x_ret_val_t ads119x_get_rld_sens_register (uint8_t * reg_rld_sens_data);

void ads119x_enable_rld_buffer (void);
void ads119x_disable_rld_buffer (void);
void ads119x_enable_rld_lead_off_sense (void);
void ads119x_disable_rld_lead_off_sense (void);
void ads119x_enable_rld_ch1_neg (void);
void ads118x_disable_rld_ch1_neg (void);
void ads119x_enable_rld_ch1_pos (void);
void ads119x_disable_rld_ch1_pos (void);

void ads119x_enable_rld_ch2_neg (void);
void ads118x_disable_rld_ch2_neg (void);
void ads119x_enable_rld_ch2_pos (void);
void ads119x_disable_rld_ch2_pos (void);

//LOFF_SENS register
ads119x_ret_val_t ads119x_get_loff_sens_register (uint8_t * reg_loff_sens_data);

void ads119x_enable_lead_off_ch1_curr_dir (void);
void ads119x_disable_lead_off_ch1_curr_dir (void);

void ads119x_enable_lead_off_ch2_curr_dir (void);
void ads119x_disable_lead_off_ch2_curr_dir (void);

void ads119x_enable_lead_off_ch1_pos_in (void);
void ads119x_disable_lead_off_ch1_pos_in (void);
void ads119x_enable_lead_off_ch1_neg_in (void);
void ads119x_disable_lead_off_ch1_neg_in (void);

void ads119x_enable_lead_off_ch2_pos_in (void);
void ads119x_disable_lead_off_ch2_pos_in (void);
void ads119x_lead_off_ch2_neg_in_enable (void);
void ads119x_lead_off_ch2_neg_in_disable (void);

//LOFF_STAT register
ads119x_ret_val_t ads119x_get_loff_state_register (uint8_t * reg_loff_stat_data);

void ads119x_set_clock_div_4 (void);
void ads119x_set_clock_div_16 (void);

bool ads119x_get_rld_connect_status (uint8_t reg_loff_stat_data);
bool ads119x_get_ch1_neg_connect_status (uint8_t reg_loff_stat_data);
bool ads119x_get_ch1_pos_connect_status (uint8_t reg_loff_stat_data);
bool ads119x_get_ch2_neg_connect_status (uint8_t reg_loff_stat_data);
bool ads19xx_get_ch2_pos_connect_status (uint8_t reg_loff_stat_data);

//MISC1 register
ads119x_ret_val_t ads119x_get_misc1_register (uint8_t * reg_misc1_data);

//MISC2 register
ads119x_ret_val_t ads119x_get_misc2_register (uint8_t * reg_misc2_data);

void ads119x_enable_offset_calibration (void);
void ads119x_disable_offset_calibration (void);

void ads119x_set_rld_reference_external (void);
void ads119x_set_rld_reference_internal (void);


//GPIO register
ads119x_ret_val_t ads119x_get_gpio_register (uint8_t * reg_gpio_data);

void ads119x_set_gpio1_as_input (void);
void ads119x_set_gpio1_as_output (void);
void ads119x_set_gpio1_output (void);
void ads119x_reset_gpio1_output (void);
bool ads119x_get_gpio1_status (uint8_t reg_gpio_data);

void ads119x_set_gpio2_as_input (void);
void ads119x_set_gpio2_as_output (void);
void ads119x_set_gpio2_output (void);
void ads119x_reset_gpio2_output (void);
bool ads119x_get_gpio2_status (uint8_t reg_gpio_data);

//Higher-level functions
ads119x_ret_val_t ads119x_comm_interface_init (void);
ads119x_ret_val_t ads119x_device_init (void);

ads119x_ret_val_t ads119x_standby_mode_enter (void);
ads119x_ret_val_t ads119x_standby_mode_wakeup (void);

ads119x_ret_val_t ads119x_start_conversion (void);
ads119x_ret_val_t ads119x_stop_conversion (void);
ads119x_ret_val_t ads119x_channel_offset_calibration_enable (void);

ads119x_ret_val_t ads119x_read_data_continous_enable (void);
ads119x_ret_val_t ads119x_read_data_continous_disable (void);

ads119x_ret_val_t ads119x_read_data_single_shot_start (void);
