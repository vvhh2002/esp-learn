//
// Created by Victor Ho on 2019/12/4.
//

#include <stdio.h>
#include <string>
#include <thread>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
int queueSize = 10;
QueueHandle_t queue;
unsigned long startProducing, endProducing, startConsuming, endConsuming, producingTime, consumingTime;

void producerTask( void * parameter )
{
    startProducing = esp_timer_get_time();

    for( int i = 0;i<queueSize;i++ ){
        xQueueSend(queue, &i, portMAX_DELAY);
    }

    endProducing = esp_timer_get_time();

    vTaskDelete( nullptr );

}


void consumerTask( void * parameter)
{
    int element;
    startConsuming = esp_timer_get_time();
    for( int i = 0;i < queueSize;i++ ){

       if( xQueueReceive(queue, &element, portMAX_DELAY)){
           printf("%d",element);
           printf("|");
       };
    }
    endConsuming= esp_timer_get_time();
    vTaskDelete( nullptr );

}

void setup() {

    queue = xQueueCreate( queueSize, sizeof( int ) );

    if(queue == nullptr){
        printf("Error creating the queue");
    }

    xTaskCreate(
            producerTask,     /* Task function. */
            "Producer",       /* String with name of task. */
            2048,            /* Stack size in words. */
            nullptr,             /* Parameter passed as input of the task */
            10,               /* Priority of the task. */
            nullptr);            /* Task handle. */

    xTaskCreate(
            consumerTask,     /* Task function. */
            "Consumer",       /* String with name of task. */
            2048,            /* Stack size in words. */
            nullptr,             /* Parameter passed as input of the task */
            10,               /* Priority of the task. */
            nullptr);            /* Task handle. */

    producingTime = endProducing - startProducing;
    printf("Producing time: %ld \n",producingTime);

    consumingTime = endConsuming - startConsuming;
    printf("Consuming time: %ld \n",consumingTime);
}



extern "C" void app_main()
{
    printf("Hello world! this is Victor greeting! from c++ mix\n");

    std::string str("string from c++");
    auto t="str2";
    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");



    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("c++ : %s  %s\n",str.c_str(),t);
    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    setup();

    fflush(stdout);
    esp_restart();
}