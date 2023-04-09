#include "driver/i2c.h"
#include "esp_log.h"
#include "ssd1306.h"
#include "font8x8_basic.h"

#define SDA_PIN 4
#define SCL_PIN 5
#define MASTER_PORT 0
#define OLED_FREQ 100000
#define OLED_ADDRESS 0x3C
#define WRITE_BIT 0
#define READ_BIT 1
#define ACK_EN 1

#define TAG "OLED"

#define OLED_CMD_SET_HORI_ADDR_MODE     0x00    
#define OLED_CMD_SET_VERT_ADDR_MODE     0x01    
#define OLED_CMD_SET_PAGE_ADDR_MODE     0x02    
#define OLED_CMD_SET_COLUMN_RANGE       0x21    


esp_err_t i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = SCL_PIN,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    esp_err_t err = i2c_param_config(MASTER_PORT, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(MASTER_PORT, conf.mode, 0, 0, 0);
}

void ssd1306_init() {
    esp_err_t espRc;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);

    i2c_master_write_byte(cmd, OLED_CMD_SET_CHARGE_PUMP, true);
    i2c_master_write_byte(cmd, 0x14, true);

    i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP, true); // reverse left-right mapping
    i2c_master_write_byte(cmd, OLED_CMD_SET_COM_SCAN_MODE, true); // reverse up-bottom mapping

    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_NORMAL, true);
    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_OFF, true);
    i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_ON, true);
    i2c_master_stop(cmd);

    espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
    if (espRc == ESP_OK) {
        ESP_LOGI(TAG, "OLED configured successfully");
    }
    else {
        ESP_LOGE(TAG, "OLED configuration failed. code: 0x%.2X", espRc);
    }
    i2c_cmd_link_delete(cmd);
}

void app_main() {

}