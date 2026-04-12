idf_component_register(SRCS "lcd_controller.c" "hd44780.c"
                       INCLUDE_DIRS "include"
                       REQUIRES freertos driver system_core hardware controllers)
