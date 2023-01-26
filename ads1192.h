#include <stdint.h>
#include <stdbool.h>

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

/*Helper definitions*/
#define BIT_POS_0   (0)
#define BIT_POS_1   (1)
#define BIT_POS_2   (2)
#define BIT_POS_3   (3)
#define BIT_POS_4   (4)
#define BIT_POS_5   (5)
#define BIT_POS_6   (6)
#define BIT_POS_7   (7)

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
    DISABLE,
    ENABLE

} enable_t;

typedef enum
{
    ADS119X_CH1,
    ADS119X_CH2

} ads119x_ch_idx_t;


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
typedef ads119x_ret_val_t (*ads119x_start_control) (uint8_t start_pin_state);
typedef ads119x_ret_val_t (*ads119x_reset) (uint8_t reset_pin_state);
typedef ads119x_ret_val_t (*ads119x_time_delay) (uint32_t time_delay_period);

typedef struct
{
    ads119x_spi_read f_dev_spi_read; 
    ads119x_spi_write f_dev_spi_write;
    ads119x_start_control f_dev_start_control;
    ads119x_reset f_dev_reset;
    ads119x_time_delay f_dev_time_delay;

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

void ads119x_control_lead_off_comparator (enable_t command);
bool ads119x_get_lead_off_comparator_status (const uint8_t reg_config_2_data);

void ads119x_control_reference_buffer (enable_t command);
bool ads119x_get_reference_buffer_status (const uint8_t reg_config_2_data);

void ads119x_set_volt_reference (ads119x_reference_t volt_reference_val);
ads119x_reference_t ads119x_get_volt_reference (const uint8_t reg_config_2_data);

void ads119x_control_clock_osc_out (enable_t command);
bool ads119x_get_clock_osc_out_status (const uint8_t reg_config_2_data);

void ads119x_control_test_signal (enable_t command);
bool ads119x_get_test_signal_status (const uint8_t reg_config_2_data);

void ads119x_set_test_signalfreq_dc (void);
void ads119x_set_test_signalfreq_1hz (void);
bool ads119x_get_test_signalfreq_status (const uint8_t reg_config_2_data);

//LOFF register
ads119x_ret_val_t ads119x_get_loff_register (uint8_t * reg_loff_data);

void ads119x_set_lead_off_comparator_thres (ads119x_lead_off_comp_pos_side_val_t comp_pos_val);
ads119x_lead_off_comp_pos_side_val_t ads119x_get_lead_off_comparator_pos_thres (const uint8_t reg_loff_data);
ads119x_lead_off_comp_neg_side_val_t ads119x_get_lead_off_comparator_neg_thres (const uint8_t reg_loff_data);

void ads119x_set_lead_off_current_val (ads119x_lead_off_curr_val_t lead_off_curr_val);
ads119x_lead_off_curr_val_t ads119x_get_lead_off_current_val (const uint8_t reg_loff_data);

void ads119x_set_lead_off_freq_dc (void);
void ads119x_set_lead_off_freq_ac (void);
bool ads119x_get_lead_off_freq_status (const uint8_t reg_loff_data);

//CH1SET and CH2SET registers
ads119x_ret_val_t ads119x_get_ch1set_register (uint8_t * reg_ch1set_data);
ads119x_ret_val_t ads119x_get_ch2set_register (uint8_t * reg_ch2set_data);


void ads119x_set_ch_power_down (ads119x_ch_idx_t ch_idx);
void ads119x_set_ch_normal_operation (ads119x_ch_idx_t ch_idx);
bool ads119x_get_ch_power_status (const uint8_t reg_ch_x_set_data, ads119x_ch_idx_t ch_idx);

void ads119x_set_ch_pga_gain_val (ads119x_ch_idx_t ch_idx, ads119x_channel_pga_gain_t pga_gain_val);
ads119x_channel_pga_gain_t ads119x_get_ch_pga_gain_val (const uint8_t reg_ch_x_set_data, ads119x_ch_idx_t ch_idx);

void ads119x_set_ch_input_source (ads119x_ch_idx_t ch_idx, ads119x_ch_input_source_t ch_input_source);
ads119x_ch_input_source_t ads119x_get_ch_input_source (const uint8_t reg_ch_x_set_data, ads119x_ch_idx_t ch_idx);


//RLD_SENS register
ads119x_ret_val_t ads119x_get_rld_sens_register (uint8_t * reg_rld_sens_data);

void ads119x_control_rld_buffer (enable_t command);
bool ads119x_get_rld_buffer_status (const uint8_t reg_rld_sens_data);

void ads119x_control_rld_lead_off_sense (enable_t command);
bool ads119x_get_rld_lead_off_sense_status (const uint8_t reg_rld_sens_data);

void ads119x_control_rld_ch_neg (ads119x_ch_idx_t ch_idx, enable_t command);
bool ads119x_get_rld_ch_neg_status (const uint8_t reg_rld_sens_data, ads119x_ch_idx_t ch_idx);

void ads119x_control_rld_ch_pos (ads119x_ch_idx_t ch_idx, enable_t command);
bool ads119x_get_rld_ch_pos_status (const uint8_t reg_rld_sens_data, ads119x_ch_idx_t ch_idx);


//LOFF_SENS register
ads119x_ret_val_t ads119x_get_loff_sens_register (uint8_t * reg_loff_sens_data);

void ads119x_control_lead_off_curr_dir (ads119x_ch_idx_t ch_idx, uint8_t command);
bool ads119x_get_lead_off_curr_dir_status (const uint8_t reg_loff_sens_data, ads119x_ch_idx_t ch_idx);

void ads119x_control_lead_off_pos_in (ads119x_ch_idx_t ch_idx, uint8_t command);
bool ads119x_get_lead_off_pos_in_status (const uint8_t reg_loff_sens_data, ads119x_ch_idx_t ch_idx);

void ads119x_control_lead_off_neg_in (ads119x_ch_idx_t ch_idx, uint8_t command);
bool ads119x_get_lead_off_neg_in_status (const uint8_t reg_loff_sens_data, ads119x_ch_idx_t ch_idx);

//LOFF_STAT register
ads119x_ret_val_t ads119x_get_loff_state_register (uint8_t * reg_loff_stat_data);

void ads119x_set_clock_div_4 (void);
void ads119x_set_clock_div_16 (void);
bool ads119x_get_clock_div_status (const uint8_t reg_loff_stat_data);

bool ads119x_get_rld_connect_status (uint8_t reg_loff_stat_data);

bool ads119x_get_pos_elect_connect_status (const uint8_t reg_loff_stat_data, ads119x_ch_idx_t ch_idx);
bool ads119x_get_neg_elect_connect_status (const uint8_t reg_loff_stat_data, ads119x_ch_idx_t ch_idx);


//MISC1 register
ads119x_ret_val_t ads119x_get_misc1_register (uint8_t * reg_misc1_data);

//MISC2 register
ads119x_ret_val_t ads119x_get_misc2_register (uint8_t * reg_misc2_data);

void ads119x_control_offset_calibration (enable_t command);
bool ads119x_get_offset_calibration_status (const uint8_t reg_misc2_data);

void ads119x_set_rld_reference_external (void);
void ads119x_set_rld_reference_internal (void);
bool ads119x_get_rld_references_status (const uint8_t reg_misc2_data);


//GPIO register
ads119x_ret_val_t ads119x_get_gpio_register (uint8_t * reg_gpio_data);

void ads119x_set_gpio1_as_input (void);
void ads119x_set_gpio1_as_output (void);
void ads119x_control_gpio1_output (enable_t command);
bool ads119x_get_gpio1_status (const uint8_t reg_gpio_data);

void ads119x_set_gpio2_as_input (void);
void ads119x_set_gpio2_as_output (void);
void ads119x_control_gpio2_output (enable_t command);
bool ads119x_get_gpio2_status (const uint8_t reg_gpio_data);

/*Higher-level functions*/

//SPI write and read functions, start and reset function must be implemented by the user.
ads119x_ret_val_t port_spi_write (uint8_t * send_buffer, uint8_t len);
ads119x_ret_val_t port_spi_read (uint8_t * recv_buffer, uint8_t len);
ads119x_ret_val_t port_start_pin_ctrl (uint8_t start_pin_state);
ads119x_ret_val_t port_reset_pin_ctrl (uint8_t reset_pin_state);
ads119x_ret_val_t port_time_delay (uint32_t time_delay_period);


void ads119x_init_comm_interface (ads119x_config_t * dev_config);
ads119x_ret_val_t ads119x_init_device (ads119x_config_t * dev_config);

ads119x_ret_val_t ads119x_standby_mode_enter (void);
ads119x_ret_val_t ads119x_standby_mode_wakeup (ads119x_config_t * dev_config);

void ads119x_reset_registers_to_default (ads119x_config_t * dev_config);

void ads119x_start_conversion (ads119x_config_t * dev_config);
void ads119x_stop_conversion (ads119x_config_t * dev_config);

void ads119x_read_data (ads119x_config_t * dev_config, uint16_t * data);

ads119x_ret_val_t ads119x_enable_read_data_continous (void);
ads119x_ret_val_t ads119x_disable_read_data_continous (void);

ads119x_ret_val_t ads119x_start_read_data_single_shot (void);
