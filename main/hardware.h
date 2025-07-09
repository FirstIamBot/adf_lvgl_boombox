#pragma once


// Defines
#define LV_TICK_PERIOD_MS 10
#define MONITOR_HEAP 1
#define GUI_TASK_STACK_SIZE 8 * 1024
#define GUI_TASK_PRIORITY 10
#define GUI_TASK_CORE 1

#define LV_HOR_RES_MAX 320
#define LV_VER_RES_MAX 240

//LCD pins 
#define LCD_HOST SPI1_HOST
#define PIN_NUM_LCD_PCLK  18
#define PIN_NUM_LCD_MOSI  23
#define PIN_NUM_LCD_CS  3
#define PIN_NUM_LCD_DC  2
#define PIN_NUM_LCD_RST 4



#define CONFIG_LCD_PIXEL_CLOCK_HZ (40 * 1000 * 1000)
#define CONFIG_LCD_CMD_BITS       (8)
#define CONFIG_LCD_PARAM_BITS     (8)
#define CONFIG_LCD_SPI            SPI2_HOST
#define CONFIG_LCD_BACKLIGHT      (gpio_num_t) GPIO_NUM_27
#define CONFIG_LCD_SPI_CLK        (gpio_num_t) GPIO_NUM_14
#define CONFIG_LCD_SPI_MOSI       (gpio_num_t) GPIO_NUM_13
#define CONFIG_LCD_SPI_MISO       (gpio_num_t) GPIO_NUM_12
#define CONFIG_LCD_DC             (gpio_num_t) GPIO_NUM_2
#define CONFIG_LCD_CS             (gpio_num_t) GPIO_NUM_15
#define CONFIG_LCD_RESET          (gpio_num_t) GPIO_NUM_NC /* GPIO_NUM_4 */
#define CONFIG_LCD_BUSY           (gpio_num_t) GPIO_NUM_NC /* GPIO_NUM_35 */

#define CONFIG_TOUCH_CLOCK_HZ ESP_LCD_TOUCH_SPI_CLOCK_HZ
#define CONFIG_TOUCH_SPI      SPI3_HOST
#define CONFIG_TOUCH_SPI_CLK  (gpio_num_t) GPIO_NUM_14
#define CONFIG_TOUCH_SPI_MOSI (gpio_num_t) GPIO_NUM_13
#define CONFIG_TOUCH_SPI_MISO (gpio_num_t) GPIO_NUM_12
#define CONFIG_TOUCH_CS       (gpio_num_t) GPIO_NUM_33
#define CONFIG_TOUCH_DC       (gpio_num_t) GPIO_NUM_NC
#define CONFIG_TOUCH_RST      (gpio_num_t) GPIO_NUM_NC
#define CONFIG_TOUCH_IRQ      (gpio_num_t) GPIO_NUM_36 /* GPIO_NUM_36 */

#define TOUCH_X_RES_MIN 17
#define TOUCH_X_RES_MAX 291

#define TOUCH_Y_RES_MIN 15
#define TOUCH_Y_RES_MAX 218
