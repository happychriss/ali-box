idf_component_register(SRCS "ali-box.c"
        INCLUDE_DIRS "."
        REQUIRES driver esp_lcd hdc2080)

lvgl_port_create_c_image("images/esp_logo.png" "images/" "ARGB8888" "NONE")
lvgl_port_add_images(${COMPONENT_LIB} "images/")