#include <Arduino.h>

#include "lidar.h"
#include "map.h"

#define VERBOSE_DEBUG 0

static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

Lidar::Lidar() {
    // Add your initialization code here
    //lidarframe_buffer = xQueueCreateStatic(max_buffered_frames, sizeof(lidarframe_t), (uint8_t*)lidarframe_buffer_data, &lidarframe_buffer_staticqueue);
    lidarframe_queue = xQueueCreate(max_buffered_frames, sizeof(lidarframe_t));
}

Lidar::~Lidar() {
    // Add your destruction code here
}

Lidar::lidarframe_t Lidar::get_lidar_frame() {
    lidarframe_t frame;
    //taskENTER_CRITICAL(&spinlock);
    //if(!uxQueueMessagesWaiting(lidarframe_buffer)) {
        if(xQueueReceive(lidarframe_queue, &frame, 10) == pdTRUE) {
            //taskEXIT_CRITICAL(&spinlock);
            // Serial.print("R(");
            // Serial.print(xPortGetCoreID());
            // Serial.print(" ");
            // Serial.print(uxQueueMessagesWaiting(lidarframe_queue));
            // Serial.print(")");
            return frame;
        }else{
            //taskEXIT_CRITICAL(&spinlock);
            return frame;
        }
}

uint8_t Lidar::calculate_crc(uint8_t *data, uint32_t len) {
    uint8_t crc = 0x00;
    for(uint32_t i = 0; i < len; i++) {
        crc = crc_table[(crc ^ *data++) & 0xFF];
    }
    return crc;
}

void Lidar::tick() {
    uint8_t byte_in;

    uint32_t bytes_available = Serial1.available();

    if(bytes_available == 0) {
        //Serial.print("...");
        vTaskDelay(1 / portTICK_PERIOD_MS);
        return;
    }

    for(uint32_t i = 0; i < bytes_available; i++) {
        byte_in = Serial1.read();
        // Serial.print("(");
        // Serial.print(byte_in, HEX);
        // Serial.print(") ");
        switch(current_state) {
            case state::WAITING_FOR_HEADER:
                if(byte_in == 0x54) {
#if VERBOSE_DEBUG == 2
                    Serial.print("[0x54] ");
#endif
                    current_frame.header = byte_in;
                    current_state = state::WAITING_FOR_VERLEN;
                }
                break;
            case state::WAITING_FOR_VERLEN:
                if(byte_in == 0x2C) {
#if VERBOSE_DEBUG == 2
                    Serial.print("[0x2C] ");
#endif
                    current_frame.verlen = 0x2C;
                    current_frame.speed = 0;
                    current_state = state::WAITING_FOR_SPEED_LSB;
                }else{
#if VERBOSE_DEBUG == 2
                    Serial.print(" Abort\n\r");
#endif
                    current_state = state::WAITING_FOR_HEADER;
                }
                break;
            case state::WAITING_FOR_SPEED_LSB:
                current_frame.speed = byte_in;
                current_state = state::WAITING_FOR_SPEED_MSB;
                break;
            case state::WAITING_FOR_SPEED_MSB:
                current_frame.speed |= (byte_in << 8);
#if VERBOSE_DEBUG == 2
                Serial.print("[SPEED ");
                Serial.print(current_frame.speed);
                Serial.print(" deg/s] ");
#endif
                current_state = state::WAITING_FOR_START_ANGLE_LSB;
                break;
            case state::WAITING_FOR_START_ANGLE_LSB:
                current_frame.start_angle = byte_in;
                current_state = state::WAITING_FOR_START_ANGLE_MSB;
                break;
            case state::WAITING_FOR_START_ANGLE_MSB:
                current_frame.start_angle |= (byte_in << 8);
#if VERBOSE_DEBUG == 2
                Serial.print("[START ANGLE ");
                Serial.print(current_frame.start_angle/100);
                Serial.print(" deg] ");
#endif
                current_state = state::WAITING_FOR_DATA_DISTANCE_LSB;
                current_frame_data_idx = 0;
                memset(current_frame.data, 0, sizeof(current_frame.data));
                break;
                /* DATA */
            case state::WAITING_FOR_DATA_DISTANCE_LSB:
                current_frame.data[current_frame_data_idx].distance = byte_in;
                current_state = state::WAITING_FOR_DATA_DISTANCE_MSB;
                break;
            case state::WAITING_FOR_DATA_DISTANCE_MSB:
                current_frame.data[current_frame_data_idx].distance |= (byte_in << 8);
                current_state = state::WAITING_FOR_DATA_INTENSITY;
                break;
            case state::WAITING_FOR_DATA_INTENSITY:
                current_frame.data[current_frame_data_idx].intensity = byte_in;
#if VERBOSE_DEBUG == 1
                Serial.print("[DATA ");
                Serial.print(current_frame_data_idx);
                Serial.print(" ");
                Serial.print(current_frame.data[current_frame_data_idx].distance);
                Serial.print(" mm ");
                Serial.print(current_frame.data[current_frame_data_idx].intensity);
                Serial.print(" mm] ");
#endif
                if(++current_frame_data_idx >= point_per_pack) {
                    current_state = state::WAITING_FOR_END_ANGLE_LSB;
                }else{
                    current_state = state::WAITING_FOR_DATA_DISTANCE_LSB;
                }
                break;
                /* END DATA */
            case state::WAITING_FOR_END_ANGLE_LSB:
                current_frame.end_angle = byte_in;
                current_state = state::WAITING_FOR_END_ANGLE_MSB;
                break;
            case state::WAITING_FOR_END_ANGLE_MSB:
#if VERBOSE_DEBUG == 2
                Serial.print("[END ANGLE ");
                Serial.print(current_frame.end_angle/100);
                Serial.print(" deg] ");
#endif
                current_frame.end_angle |= (byte_in << 8);
                current_state = state::WAITING_FOR_TIMESTAMP_LSB;
                break;
            case state::WAITING_FOR_TIMESTAMP_LSB:
                current_frame.timestamp = byte_in;
                current_state = state::WAITING_FOR_TIMESTAMP_MSB;
                break;
            case state::WAITING_FOR_TIMESTAMP_MSB:
#if VERBOSE_DEBUG == 2
                Serial.print("[TIMESTAMP ");
                Serial.print(current_frame.timestamp);
                Serial.print(" ms] ");
#endif
                current_frame.timestamp |= (byte_in << 8);
                current_state = state::WAITING_FOR_CRC;
                break;
            case state::WAITING_FOR_CRC:
                current_frame.crc_8 = byte_in;
                uint8_t calculated_crc = calculate_crc((uint8_t*)&current_frame, sizeof(current_frame) - 1);
#if VERBOSE_DEBUG == 1
                Serial.print("[CRC] ");
                Serial.print(calculated_crc, HEX);
                Serial.print(" vs ");
                Serial.print(current_frame.crc_8, HEX);
                Serial.print(" ");
#endif
                if(calculated_crc == current_frame.crc_8) {
#if VERBOSE_DEBUG == 1
                    Serial.print("[CRC OK]\n\r");
#endif
                    //taskENTER_CRITICAL(&spinlock);
                    if(xQueueSend(lidarframe_queue, &current_frame, 0) == pdTRUE) {
                        // Serial.print("S(");
                        // Serial.print(uxQueueMessagesWaiting(lidarframe_queue));
                        // Serial.print(")");
                    }else{
                        Serial.print("x");
                    }
                    //taskEXIT_CRITICAL(&spinlock);
                }else{
                    Serial.print("[CRC FAIL]\n\r");
                }
                //Serial.println();
                current_state = state::WAITING_FOR_HEADER;
                break;
        }
    }
    }
