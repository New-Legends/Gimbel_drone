
#include "dmimu_task.h"


void dmimu_task(void *pvParameters)
{
    DM_IMU_INIT();
    while (1)
    {
        IMU_run();
        vTaskDelay(DMIMU_CONTROL_TIME_MS);
        /* code */
    }
    
}