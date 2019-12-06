//
// Created by Victor Ho on 2019/12/6.
//

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
static const char* TAG = "main";

extern "C" void app_main()
{
    ESP_LOGI(TAG,"start !");


    ESP_LOGI(TAG,"end !");
}