/*********************
 *      INCLUDES
 *********************/
#include "lvgl.h"
#include "lv_demos.h"
#include <stdio.h>
#include "gui.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_freertos_hooks.h"
#include "esp_log.h"

#include "encoder.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "disp_driver.h"

 /*********************
 *      DEFINES
 *********************/
#define TAG "gui"
#define RE_A_GPIO   CONFIG_RE_A_GPIO    // 16
#define RE_B_GPIO   CONFIG_RE_B_GPIO    //17
#define RE_BTN_GPIO CONFIG_RE_BTN_GPIO  //9
#define EV_QUEUE_LEN 5
/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**********************
 *  VARIABLES
 **********************/
lv_indev_t * indev_touchpad;
static lv_indev_drv_t touch_indev_drv;

lv_indev_t * indev_encoder;
static lv_indev_drv_t encoder_indev_drv;

// Define Semaphore
SemaphoreHandle_t xGuiSemaphore;    // Семафор для работы lvgl

// Define Queue
extern QueueHandle_t xQueueGUItoBoombox;  // Очередь для передачи с GUI на Boombox
extern QueueHandle_t xQueueBoomboxtoGUI;  // Очередь для передачи с Boombox на GUI


TaskHandle_t TaskTFT;
TaskHandle_t TaskAudio;
TaskHandle_t TaskRadio;
TaskHandle_t *TaskBT;

extern int slider_vol;
rotary_encoder_event_t e;
int32_t new_value;

static QueueHandle_t event_queue;
static rotary_encoder_t re;


/**************************************************************************************
*                             Prototype function
**************************************************************************************/

void encoder_init(void) {

	// Create event queue for rotary encoders
    event_queue = xQueueCreate(EV_QUEUE_LEN, sizeof(rotary_encoder_event_t));

    // Setup rotary encoder library
    ESP_ERROR_CHECK(rotary_encoder_init(event_queue));
    
    // Add one encoder
    memset(&re, 0, sizeof(rotary_encoder_t));
    re.pin_a = RE_A_GPIO;
    re.pin_b = RE_B_GPIO;
    re.pin_btn = RE_BTN_GPIO;
    ESP_ERROR_CHECK(rotary_encoder_add(&re));

}


//************************************************************************************ */

/**
 * @ingroup group18 Covert numbers to char array
 * @brief Converts a number to a char array
 * @details It is useful to mitigate memory space used by functions like sprintf or other generic similar functions
 * @details You can use it to format frequency using decimal or thousand separator and also to convert small numbers.
 *
 * @param value  value to be converted
 * @param strValue char array that will be receive the converted value
 * @param len final string size (in bytes)
 * @param dot the decimal or thousand separator position
 * @param separator symbol "." or ","
 * @param remove_leading_zeros if true removes up to two leading zeros (default is true)
 */
void ConvertToChar(uint16_t value, char *strValue, uint8_t len, uint8_t dot, uint8_t separator, bool remove_leading_zeros)
{
    char d;
    for (int i = (len - 1); i >= 0; i--)
    {
        d = value % 10;
        value = value / 10;
        strValue[i] = d + 48;
    }
    strValue[len] = '\0';
    if (dot > 0)
    {
        for (int i = len; i >= dot; i--)
        {
            strValue[i + 1] = strValue[i];
        }
        strValue[dot] = separator;
    }

    if (remove_leading_zeros)
    {
        if (strValue[0] == '0')
        {
            strValue[0] = ' ';
            if (strValue[1] == '0')
                strValue[1] = ' ';
        }
    }
}

static void lv_tick_task(void)
{
   lv_tick_inc(portTICK_PERIOD_MS);
}



/**************************************************************************************
 *                            Task function
 **************************************************************************************/
void task_gui(void *arg)
{
   xGuiSemaphore = xSemaphoreCreateMutex();
   lv_init();          
   lvgl_driver_init(); 

   /* Example for 1) */
   static lv_disp_draw_buf_t draw_buf;
   lv_color_t *buf1 = heap_caps_malloc((LV_HOR_RES_MAX * LV_VER_RES_MAX/10) * sizeof(lv_color_t), MALLOC_CAP_DMA);
   lv_color_t *buf2 = heap_caps_malloc((LV_HOR_RES_MAX * LV_VER_RES_MAX/10) * sizeof(lv_color_t), MALLOC_CAP_DMA);

   lv_disp_draw_buf_init(&draw_buf, buf1, buf2, (LV_HOR_RES_MAX * LV_VER_RES_MAX/10)); /*Initialize the display buffer*/
   /*------------------
   * Display
   * -----------------*/
   static lv_disp_drv_t disp_drv;         /*A variable to hold the drivers. Must be static or global.*/
   lv_disp_drv_init(&disp_drv);           /*Basic initialization*/
   disp_drv.draw_buf = &draw_buf;         /*Set an initialized buffer*/
   disp_drv.flush_cb = disp_driver_flush; /*Set a flush callback to draw to the display*/
   disp_drv.hor_res = LV_HOR_RES_MAX;                /*Set the horizontal resolution in pixels*/
   disp_drv.ver_res = LV_VER_RES_MAX;                /*Set the vertical resolution in pixels*/
   lv_disp_drv_register(&disp_drv);       /*Register the driver and save the created display objects*/
   /*------------------
   * TouchPad
   * -----------------*/
   lv_indev_drv_init( &touch_indev_drv );
   touch_indev_drv.type = LV_INDEV_TYPE_POINTER;
   touch_indev_drv.read_cb = touch_driver_read;
   indev_touchpad = lv_indev_drv_register(&touch_indev_drv);

   esp_register_freertos_tick_hook(lv_tick_task);

    //lv_demo_music();
    lv_demo_benchmark();

   while (1)
   {
      /* Delay 1 tick (assumes FreeRTOS tick is 10ms */
      vTaskDelay(pdMS_TO_TICKS(10));
      /* Try to take the semaphore, call lvgl related function on success */
      if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY))
      {
         lv_timer_handler();
         xSemaphoreGive(xGuiSemaphore);
      }
   }
}

