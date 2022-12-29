#include <stdint.h>


#ifndef ENABLE
#define ENABLE 1
#endif

#ifndef DISABLE
#define DISABLE 0
#endif

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


typedef struct
{
    //TODO: Add function pointers to read and write functions.

} ads119x_config_t;
typedef struct
{
    uint16_t ch1_raw_data;
    uint16_t ch2_raw_data;

} ads119x_meas_t;


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
    ADS119X_DATA_RATE_8000_SPS = 6

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
    ADS119X_PGA_GAIN_1 = 0,
    ADS119X_PGA_GAIN_2,
    ADS119X_PGA_GAIN_3,
    ADS119X_PGA_GAIN_4,
    ADS119X_PGA_GAIN_6,
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

ads119x_ret_val_t ads119x_get_device_id (ads119x_device_id_t * ads119x_device_id);

ads119x_ret_val_t ads119x_get_data_rate (ads119x_data_rate_t * ads119x_data_rate);
ads119x_ret_val_t ads119x_set_data_rate (ads119x_data_rate_t ads119x_data_rate);

ads119x_ret_val_t ads119x_get_conversion_mode (asd119x_conversion_mode_t * conv_mode);
ads119x_ret_val_t ads119x_set_conversion_mode (asd119x_conversion_mode_t conv_mode);

ads119x_ret_val_t ads119x_lead_off_comparator_enable (void);
ads119x_ret_val_t ads119x_lead_off_comparator_disable (void);

ads119x_ret_val_t ads119x_reference_buffer_enable (void);
ads119x_ret_val_t ads119x_reference_buffer_disable (void);

ads119x_ret_val_t ads119x_set_volt_reference (ads119x_reference_t volt_reference_val);

ads119x_ret_val_t ads119x_clock_osc_out_enable (void);
ads119x_ret_val_t ads119x_clock_osc_out_disable (void);

ads119x_ret_val_t ads119x_test_signal_enable (void);
ads119x_ret_val_t ads119x_test_signal_disable (void);
ads119x_ret_val_t ads119x_test_signal_set_freq_dc (void);
ads119x_ret_val_t ads119x_test_signal_set_freq_1hz (void);

ads119x_ret_val_t ads119x_lead_off_comparator_set_pos_val (ads119x_lead_off_comp_pos_side_val_t comp_pos_val);
ads119x_ret_val_t ads119x_lead_off_comparator_set_neg_val (ads119x_lead_off_comp_neg_side_val_t comp_neg_val);

ads119x_ret_val_t ads119x_lead_off_current_set_val (ads119x_lead_off_curr_val_t lead_off_curr_val);
ads119x_ret_val_t ads119x_lead_off_freq_dc (void);
ads119x_ret_val_t ads119x_lead_off_freq_ac (void);


ads119x_ret_val_t ads119x_ch1_power_down (void);
ads119x_ret_val_t ads119x_ch1_normal_operation (void);
ads119x_ret_val_t ads119x_ch1_pga_gain_set_val (ads119x_channel_pga_gain_t pga_gain_val);
ads119x_ret_val_t ads119x_ch1_input_source_select (ads119x_ch_input_source_t ch_input_source);

ads119x_ret_val_t ads119x_ch2_power_down (void);
ads119x_ret_val_t ads119x_ch2_normal_operation (void);
ads119x_ret_val_t ads119x_ch2_pga_gain_set_val (ads119x_channel_pga_gain_t pga_gain_val);
ads119x_ret_val_t ads119x_ch2_input_source_select (ads119x_ch_input_source_t ch_input_source);

ads119x_ret_val_t ads119x_rld_buffer_enable (void);
ads119x_ret_val_t ads119x_rld_buffer_disable (void);
ads119x_ret_val_t ads119x_rld_lead_off_sense_enable (void);
ads119x_ret_val_t ads119x_rld_lead_off_sense_disable (void);
ads119x_ret_val_t ads119x_rld_ch1_neg_enable (void);
ads118x_ret_val_t ads118x_rld_ch1_neg_disable (void);
ads119x_ret_val_t ads119x_rld_ch1_pos_enable (void);
ads119x_ret_val_t ads119x_rld_ch1_pos_disable (void);

ads119x_ret_val_t ads119x_rld_ch2_neg_enable (void);
ads118x_ret_val_t ads118x_rld_ch2_neg_disable (void);
ads119x_ret_val_t ads119x_rld_ch2_pos_enable (void);
ads119x_ret_val_t ads119x_rld_ch2_pos_disable (void);

ads119x_ret_val_t ads119x_lead_off_ch1_curr_dir_enable (void);
ads119x_ret_val_t ads119x_lead_off_ch1_curr_dir_disable (void);

ads119x_ret_val_t ads119x_lead_off_ch2_curr_dir_enable (void);
ads119x_ret_val_t ads119x_lead_off_ch2_curr_dir_disable (void);

ads119x_ret_val_t ads119x_lead_off_ch1_pos_in_enable (void);
ads119x_ret_val_t ads119x_lead_off_ch1_pos_in_disable (void);
ads119x_ret_val_t ads119x_lead_off_ch1_neg_in_enable (void);
ads119x_ret_val_t ads119x_lead_off_ch1_neg_in_disable (void);

ads119x_ret_val_t ads119x_lead_off_ch2_pos_in_enable (void);
ads119x_ret_val_t ads119x_lead_off_ch2_pos_in_disable (void);
ads119x_ret_val_t ads119x_lead_off_ch1_neg_in_enable (void);
ads119x_ret_val_t ads119x_lead_off_ch1_neg_in_disable (void);

ads119x_ret_val_t ads119x_set_clock_div_4 (void);
ads119x_ret_val_t ads119x_set_clock_div_16 (void);

ads119x_ret_val_t ads119x_get_rld_status (void);
ads119x_ret_val_t ads119x_rld_enable (void);
ads119x_ret_val_t ads119x_rld_disable (void);

ads119x_ret_val_t ads119x_ch1_pos_in_enable (void);
ads119x_ret_val_t ads119x_ch1_neg_in_disable (void);
ads119x_ret_val_t ads119x_ch1_neg_in_enable (void);
ads119x_ret_val_t ads119x_ch1_neg_in_disable (void);

ads119x_ret_val_t ads119x_ch2_pos_in_enable (void);
ads119x_ret_val_t ads119x_ch2_neg_in_disable (void);
ads119x_ret_val_t ads119x_ch2_neg_in_enable (void);
ads119x_ret_val_t ads119x_ch2_neg_in_disable (void);

ads119x_ret_val_t ads119x_offset_calibration_enable (void);
ads119x_ret_val_t ads119x_offset_calibration_disable (void);

ads119x_ret_val_t ads119x_rld_reference_set_external (void);
ads119x_ret_val_t ads119x_rld_reference_set_internal (void);
ads119x_ret_val_t ads119x_rld_referenc_get (void);

ads119x_ret_val_t ads119x_gpio1_config (void);
ads119x_ret_val_t ads119x_gpio1_set (void);
ads119x_ret_val_t ads119x_gpio1_reset (void);
ads119x_ret_val_t ads119x_gpio1_get (void);

ads119x_ret_val_t ads119x_gpio2_config (void);
ads119x_ret_val_t ads119x_gpio2_set (void);
ads119x_ret_val_t ads119x_gpio2_reset (void);
ads119x_ret_val_t ads119x_gpio2_get (void);