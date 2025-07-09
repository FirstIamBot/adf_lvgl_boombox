#include "lvgl.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include "global.h"
//#include "lvgl_helpers.h"
//include "ui.h"
//#include "sd_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "esp_err.h"
#include "hardware.h"

#include "gui.h"


#if LV_USE_DEMO_WIDGETS
#include "lv_demos.h"
#endif

QueueHandle_t xQueueGUItoBoombox;  // Очередь для передачи с GUI на Boombox
QueueHandle_t xQueueBoomboxtoGUI;  // Очередь для передачи с Boombox на GUI


static const char *TAG = " ************** main_LVGL";

// global vars
SemaphoreHandle_t xGuiSemaphore;


// Static Prototypes

void app_main()
{
 task_gui(NULL);
}

