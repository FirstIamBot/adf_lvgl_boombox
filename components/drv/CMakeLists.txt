
idf_component_register(SRCS 
                            "disp_driver.c"
                            
                       INCLUDE_DIRS . ./tft ./touch
                       REQUIRES lvgl driver)