//
// Created by Victor Ho on 2019/12/4.
//

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#define STACK_SIZE 2048
//extern struct tskTaskControlBlock* volatile pxCurrentTCB;
// Task to be created.
void vTaskCode( void * pvParameters )
{
    for (int i = 10; i >= 0; i--) {
        printf("task stop in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void task(void *pvPar)
{
    while(1)
    {
        printf("I'm %s\r\n",(char *)pvPar);
        //使用此延时API可以将任务转入阻塞态，期间CPU继续运行其它任务
       vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(){
    printf("Start Task test!!\n");

    vTaskDelay(pdMS_TO_TICKS(100));//等待系统初始化
    //以下创建了三个任务，使用的任务函数都是task()，只不过传入的参数不同，每一个任务独立运行
    xTaskCreatePinnedToCore(task,           //任务函数
                            "task1",         //这个参数没有什么作用，仅作为任务的描述
                            2048,            //任务栈的大小
                            "task1",         //传给任务函数的参数
                            2,              //优先级，数字越大优先级越高
                            NULL,            //传回的句柄，不需要的话可以为NULL
                            tskNO_AFFINITY); //指定运行任务的ＣＰＵ，使用这个宏表示不会固定到任何核上
    xTaskCreatePinnedToCore(task, "task2", 2048, "task2",2, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(task, "task3" ,2048, "task3",2, NULL, tskNO_AFFINITY);
    for(;;)
        vTaskDelay(1000 / portTICK_PERIOD_MS);


}