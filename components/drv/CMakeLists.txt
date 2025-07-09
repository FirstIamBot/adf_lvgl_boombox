
idf_component_register(SRCS 
                         "disp_driver.c"
                         "tft/disp_spi.c"
                         "tft/ili9341.c"
                         "touch/touch_driver.c"
                         "touch/tp_spi.c"
                         "touch/xpt2046.c"
                         INCLUDE_DIRS . ./tft ./touch
                         REQUIRES lvgl driver)