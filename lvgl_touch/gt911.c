/*
* Copyright © 2021 Sturnus Inc.

* Permission is hereby granted, free of charge, to any person obtaining a copy of this 
* software and associated documentation files (the “Software”), to deal in the Software 
* without restriction, including without limitation the rights to use, copy, modify, merge, 
* publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons 
* to whom the Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all copies or 
* substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
* SOFTWARE.
*/

#include <esp_log.h>
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include <lvgl.h>
#else
#include <lvgl/lvgl.h>
#endif
#include "gt911.h"

#include "lvgl_i2c/i2c_manager.h"

#define TAG "GT911"

gt911_status_t gt911_status;

//TODO: handle multibyte read and refactor to just one read transaction
esp_err_t gt911_i2c_read(uint8_t slave_addr, uint16_t register_addr, uint8_t *data_buf, uint8_t len) {
    return lvgl_i2c_read(CONFIG_LV_I2C_TOUCH_PORT, slave_addr, register_addr | I2C_REG_16, data_buf, len);
}

esp_err_t gt911_i2c_write8(uint8_t slave_addr, uint16_t register_addr, uint8_t data) {
    uint8_t buffer = data;
    return lvgl_i2c_write(CONFIG_LV_I2C_TOUCH_PORT, slave_addr, register_addr | I2C_REG_16, &buffer, 1);
}

/**
  * @brief  Initialize for GT911 communication via I2C
  * @param  dev_addr: Device address on communication Bus (I2C slave address of GT911).
  * @retval None
  */
void gt911_init(uint8_t dev_addr) {
    if (!gt911_status.inited) {
        gt911_status.i2c_dev_addr = dev_addr;
        uint8_t data_buf;
        esp_err_t ret;

        //.................................................................................
        // Cambio sensibilità touchscreen
        //.................................................................................
        uint8_t threshold_touch_detected = 0;
        uint8_t threshold_touch_released = 0;
        uint8_t pga_gain = 0;
        uint8_t panel_pga_gain = 0;
        uint8_t gesturedrv_pga_gain = 0;
        uint8_t flag_modify_configuration = 0;
        vTaskDelay(pdMS_TO_TICKS(100));

        /* Read current values */
        gt911_i2c_read(dev_addr, 0x8053, &threshold_touch_detected, 1);
        ESP_LOGI(TAG, "\tThreshold touch detected: %d", threshold_touch_detected);

        gt911_i2c_read(dev_addr, 0x8054, &threshold_touch_released, 1);
        ESP_LOGI(TAG, "\tThreshold touch released: %d", threshold_touch_released);

        gt911_i2c_read(dev_addr, 0x80A5, &pga_gain, 1);
        ESP_LOGI(TAG, "\tPGA_Gain: 0x%x", pga_gain & 0x07);

        gt911_i2c_read(dev_addr, 0x8074, &gesturedrv_pga_gain, 1);
        ESP_LOGI(TAG, "\tGestureDrv_PGA_Gain: 0x%x", gesturedrv_pga_gain & 0x0F);

        gt911_i2c_read(dev_addr, 0x806C, &panel_pga_gain, 1);
        ESP_LOGI(TAG, "\tPanel_PGA_Gain: 0x%x", panel_pga_gain & 0x07);

        /* Check if the current values are different from the setpoints and update them in that case */
        if(threshold_touch_detected != CONFIG_LV_GT911_THRSHLD_TOUCH_DETECTED)
        {
        	gt911_i2c_write8(dev_addr, 0x8053, CONFIG_LV_GT911_THRSHLD_TOUCH_DETECTED);
        	ESP_LOGI(TAG, "\tNEW! threshold touch detected: %d", CONFIG_LV_GT911_THRSHLD_TOUCH_DETECTED);
        	flag_modify_configuration = 1;
        }

        if(threshold_touch_released != CONFIG_LV_GT911_THRSHLD_TOUCH_RELEASED)
        {
        	gt911_i2c_write8(dev_addr, 0x8054, CONFIG_LV_GT911_THRSHLD_TOUCH_RELEASED);
        	ESP_LOGI(TAG, "\tNEW! threshold touch released: %d", CONFIG_LV_GT911_THRSHLD_TOUCH_RELEASED);
        	flag_modify_configuration = 1;
        }

        if((pga_gain & 0x07) != CONFIG_LV_GT911_PGA_GAIN)
        {
        	gt911_i2c_write8(dev_addr, 0x80A5, ((pga_gain & 0xF8)|CONFIG_LV_GT911_PGA_GAIN));
        	ESP_LOGI(TAG, "\tNEW! PGA_Gain: 0x%x", CONFIG_LV_GT911_PGA_GAIN);
        	flag_modify_configuration = 1;
        }

        if((gesturedrv_pga_gain & 0x0F) != CONFIG_LV_GT911_GESTUREDRV_PGA_GAIN)
        {
        	gt911_i2c_write8(dev_addr, 0x8074, ((gesturedrv_pga_gain & 0xF0)|CONFIG_LV_GT911_GESTUREDRV_PGA_GAIN));
        	ESP_LOGI(TAG, "\tNEW! GestureDrv_PGA_Gain: 0x%x", CONFIG_LV_GT911_GESTUREDRV_PGA_GAIN);
        	flag_modify_configuration = 1;
        }

        if((panel_pga_gain & 0x07) != CONFIG_LV_GT911_PANEL_PGA_GAIN)
        {
        	gt911_i2c_write8(dev_addr, 0x806C, ((panel_pga_gain & 0xF8)|CONFIG_LV_GT911_PANEL_PGA_GAIN));
        	ESP_LOGI(TAG, "\tNEW! Panel_PGA_Gain: 0x%x", CONFIG_LV_GT911_PANEL_PGA_GAIN);
        	flag_modify_configuration = 1;
        }

        /* Update configuration checksum if there have been any modification */
        if(flag_modify_configuration) //if the configuration has been modified, we need to update the checksum
        {
			uint8_t checksum = 0;
			for(int i=0; i<183; i++)
			{
				gt911_i2c_read(dev_addr, 0x8047 + i, &data_buf, 1);
				checksum += data_buf;
			}
			checksum = (checksum ^ 0xFF)+1;

			gt911_i2c_write8(dev_addr, 0x80FF, checksum);
			gt911_i2c_write8(dev_addr, 0x8100, 1);
        }

        //.................................................................................

        ESP_LOGI(TAG, "Checking for GT911 Touch Controller");
        if ((ret = gt911_i2c_read(dev_addr, GT911_PRODUCT_ID1, &data_buf, 1) != ESP_OK)) {
            ESP_LOGE(TAG, "Error reading from device: %s",
                        esp_err_to_name(ret));    // Only show error the first time
            return;
        }

        // Read 4 bytes for Product ID in ASCII
        for (int i = 0; i < GT911_PRODUCT_ID_LEN; i++) {
            gt911_i2c_read(dev_addr, (GT911_PRODUCT_ID1 + i), (uint8_t *)&(gt911_status.product_id[i]), 1);
        }
        ESP_LOGI(TAG, "\tProduct ID: %s", gt911_status.product_id);

        gt911_i2c_read(dev_addr, GT911_VENDOR_ID, &data_buf, 1);
        ESP_LOGI(TAG, "\tVendor ID: 0x%02x", data_buf);

        gt911_i2c_read(dev_addr, GT911_X_COORD_RES_L, &data_buf, 1);
        gt911_status.max_x_coord = data_buf;
        gt911_i2c_read(dev_addr, GT911_X_COORD_RES_H, &data_buf, 1);
        gt911_status.max_x_coord |= ((uint16_t)data_buf << 8);
        ESP_LOGI(TAG, "\tX Resolution: %d", gt911_status.max_x_coord);

        gt911_i2c_read(dev_addr, GT911_Y_COORD_RES_L, &data_buf, 1);
        gt911_status.max_y_coord = data_buf;
        gt911_i2c_read(dev_addr, GT911_Y_COORD_RES_H, &data_buf, 1);
        gt911_status.max_y_coord |= ((uint16_t)data_buf << 8);
        ESP_LOGI(TAG, "\tY Resolution: %d", gt911_status.max_y_coord);
        gt911_status.inited = true;
    }
}

/**
  * @brief  Get the touch screen X and Y positions values. Ignores multi touch
  * @param  drv:
  * @param  data: Store data here
  * @retval Always false
  */
bool gt911_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    uint8_t touch_pnt_cnt;        // Number of detected touch points
    static int16_t last_x = 0;  // 12bit pixel value
    static int16_t last_y = 0;  // 12bit pixel value
    uint8_t data_buf;
    uint8_t status_reg;

    gt911_i2c_read(gt911_status.i2c_dev_addr, GT911_STATUS_REG, &status_reg, 1);
//    ESP_LOGI(TAG, "\tstatus: 0x%02x", status_reg);
    touch_pnt_cnt = status_reg & 0x0F;
    if ((status_reg & 0x80) || (touch_pnt_cnt < 6)) {
        //Reset Status Reg Value
        gt911_i2c_write8(gt911_status.i2c_dev_addr, GT911_STATUS_REG, 0x00);
    }
    if (touch_pnt_cnt != 1) {    // ignore no touch & multi touch
        data->point.x = last_x;
        data->point.y = last_y;
        data->state = LV_INDEV_STATE_REL;
        return false;
    }

//    gt911_i2c_read(gt911_status.i2c_dev_addr, GT911_TRACK_ID1, &data_buf, 1);
//    ESP_LOGI(TAG, "\ttrack_id: %d", data_buf);

    gt911_i2c_read(gt911_status.i2c_dev_addr, GT911_PT1_X_COORD_L, &data_buf, 1);
    last_x = data_buf;
    gt911_i2c_read(gt911_status.i2c_dev_addr, GT911_PT1_X_COORD_H, &data_buf, 1);
    last_x |= ((uint16_t)data_buf << 8);

    gt911_i2c_read(gt911_status.i2c_dev_addr, GT911_PT1_Y_COORD_L, &data_buf, 1);
    last_y = data_buf;
    gt911_i2c_read(gt911_status.i2c_dev_addr, GT911_PT1_Y_COORD_H, &data_buf, 1);
    last_y |= ((uint16_t)data_buf << 8);

#if CONFIG_LV_GT911_INVERT_X
    last_x = gt911_status.max_x_coord - last_x;
#endif
#if CONFIG_LV_GT911_INVERT_Y
    last_y = gt911_status.max_y_coord - last_y;
#endif
#if CONFIG_LV_GT911_SWAPXY
    int16_t swap_buf = last_x;
    last_x = last_y;
    last_y = swap_buf;
#endif
    data->point.x = last_x;
    data->point.y = last_y;
    data->state = LV_INDEV_STATE_PR;
    ESP_LOGI(TAG, "X=%u Y=%u", data->point.x, data->point.y);
    ESP_LOGV(TAG, "X=%u Y=%u", data->point.x, data->point.y);
    return false;
}
