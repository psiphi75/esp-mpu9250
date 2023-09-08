/*****************************************************************************
 *                                                                           *
 *  Copyright 2018 Simon M. Werner                                           *
 *                                                                           *
 *  Licensed under the Apache License, Version 2.0 (the "License");          *
 *  you may not use this file except in compliance with the License.         *
 *  You may obtain a copy of the License at                                  *
 *                                                                           *
 *      http://www.apache.org/licenses/LICENSE-2.0                           *
 *                                                                           *
 *  Unless required by applicable law or agreed to in writing, software      *
 *  distributed under the License is distributed on an "AS IS" BASIS,        *
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. *
 *  See the License for the specific language governing permissions and      *
 *  limitations under the License.                                           *
 *                                                                           *
 *****************************************************************************/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "common.h"

void pause(void)
{

  static uint64_t start = 0;
  uint64_t end = xTaskGetTickCount() * 1000 / configTICK_RATE_HZ;

  if (start == 0)
  {
    start = xTaskGetTickCount() / configTICK_RATE_HZ;
  }

  int32_t elapsed = end - start;
  if (elapsed < SAMPLE_INTERVAL_MS)
  {
    vTaskDelay((SAMPLE_INTERVAL_MS - elapsed) / portTICK_PERIOD_MS);
  }
  start = xTaskGetTickCount() * 1000 / configTICK_RATE_HZ;
}
