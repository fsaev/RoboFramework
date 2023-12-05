#include <Arduino.h>
#include <roboframework.h>

Lidar lidar {};
WorldMap world_map {lidar};
Path path {world_map};
BLE ble {"RoboFramework", &world_map};

uint32_t loop_subroutine = 0;
//char buf[512];

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

/* Framework is spawned as a parallell tasks, so that it won't matter if you block stuff */

void roboframework_task_lidar(void *pvParameters) {
  for(;;) {
    //vTaskDelay(2000 / portTICK_PERIOD_MS); //200 Hz regulation loop
    lidar.tick();
    
    if(millis() - loop_subroutine > 1000) {
      loop_subroutine = millis();
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      //world_map.print_all_ascii();
    }
  }
}

void roboframework_task_main_core0(void *pvParameters) {
  //world_map.example_fill(std::make_pair(128, 128));
  for(;;) {

    world_map.tick_frame();
    //vTaskDelay(1 / portTICK_PERIOD_MS); //200 Hz regulation loop
  }
}

void roboframework_task_main_core1(void *pvParameters) {
  //world_map.example_fill(std::make_pair(128, 128));
  ble.start_server();
  for(;;) {

    world_map.tick_frame();
    world_map.tick_com();
    //vTaskDelay(1 / portTICK_PERIOD_MS); //200 Hz regulation loop
  }
}

void roboframework_task_path(void *pvParameters) {
  for(;;) {
    vTaskDelay(5 / portTICK_PERIOD_MS); //200 Hz regulation loop
  }
}


void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(6, OUTPUT);
  Serial.begin(230400); //USB serial
  Serial1.begin(230400, SERIAL_8N1, RX, TX);

  //world_map.enable_printing();

  xTaskCreatePinnedToCore(
                    roboframework_task_lidar,          /* Task function. */
                    "roboplatform_lidar",        /* String with name of task. */
                    10000,            /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    4,                /* Priority of the task. */
                    NULL,              /* Task handle. */
                    1);              /* Core where the task should run */
  xTaskCreatePinnedToCore(
                    roboframework_task_main_core0,          /* Task function. */
                    "roboplatform_main_core0",        /* String with name of task. */
                    10000,            /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    3,                /* Priority of the task. */
                    NULL,              /* Task handle. */
                    0);              /* Core where the task should run */
  xTaskCreatePinnedToCore(
                    roboframework_task_main_core1,          /* Task function. */
                    "roboplatform_main_core1",        /* String with name of task. */
                    10000,            /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    3,                /* Priority of the task. */
                    NULL,              /* Task handle. */
                    1);              /* Core where the task should run */
  xTaskCreatePinnedToCore(
                    roboframework_task_path,          /* Task function. */
                    "roboplatform_path",        /* String with name of task. */
                    10000,            /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    2,                /* Priority of the task. */
                    NULL,              /* Task handle. */
                    0);              /* Core where the task should run */

  Serial.println("Setup done");
}

void loop() {

  // put your main code here, to run repeatedly:
  // if(Serial1.available()) { //Forward serial1 to serial
  //   sprintf(buf, "%02x ", Serial1.read());
  //   Serial.write(buf);
  // }


}