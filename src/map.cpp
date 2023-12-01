#include "map.h"
#include <Arduino.h>
#include <string>
#include <iostream>
#include <sstream>
constexpr std::array<std::array<WorldMap::CellType, 8>, 8> WorldMap::example_map;

static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

WorldMap::WorldMap(Lidar& lidar) : _lidar(lidar) {
    for(auto& row : map) {
        for(auto& cell : row) {
            cell = CellType::EMPTY;
        }
    }

}

void WorldMap::example_fill(std::pair<uint32_t, uint32_t> top_left) {
    for(auto row = example_map.cbegin(); row != example_map.cend(); ++row) {
        for(auto cell = row->cbegin(); cell != row->cend(); ++cell) {
            map.at(top_left.first + std::distance(example_map.cbegin(), row))
                .at(top_left.second + std::distance(row->cbegin(), cell)) = *cell;
        }
    }
}

/**
 * @brief Print the map to the serial port
 * 
 */
void WorldMap::print_all_ascii() {
    if(print_map) {
        Serial.println("You must disable printing of the map before calling this function");
        return;
    }
    for(const auto& row : map) {
        for(const auto& cell : row) {
            switch(cell) {
                case CellType::EMPTY:
                    Serial.print(" ");
                    break;
                case CellType::OBSTACLE:
                    Serial.print("X");
                    break;
                case CellType::ROBOT:
                    Serial.print("R");
                    break;
                case CellType::TARGET:
                    Serial.print("T");
                    break;
            }
        }
        Serial.println();
    }
}

void WorldMap::enable_printing() {
    print_map = true;
}

void WorldMap::disable_printing() {
    print_map = false;
}

WorldMap::CellType WorldMap::get_cell(std::pair<uint32_t, uint32_t> pos) {
    return map.at(pos.first).at(pos.second);
}

void WorldMap::send_next_chunk() {
    static_assert(map_dim % chunk_size == 0, "map_dim must be divisible by chunk_size");
    CellType* p = map[0].data(); //Start of 2D-array

    std::stringstream chunk;

    if(chunk_printidx == 0) {
        chunk << "MAP " << map_dim << "/" << map_dim << ":"; //Start of map
    }

    chunk << chunk_printidx << "," << chunk_size << ",";
    for(uint32_t i = 0; i < chunk_size; ++i) {
        // Access the 2D-array as a sequential 1D-array
        switch(p[chunk_printidx + i]) {
                case CellType::EMPTY:
                    chunk << " ";
                    break;
                case CellType::OBSTACLE:
                    chunk << "1";
                    break;
                case CellType::ROBOT:
                    chunk << "2";
                    break;
                case CellType::TARGET:
                    chunk << "3";
                    break;
        }
    }
    if((chunk_printidx += chunk_size) >= (map_dim * map_dim)) {
        chunk << ";"; //End of map
        chunk_printidx = 0;
    }

    Serial.println(chunk.str().c_str());
}

std::pair<uint32_t, uint32_t> WorldMap::get_agent_pos() {
    return std::make_pair(agent_x, agent_y);
}

void WorldMap::raycast_data(uint32_t angle, uint32_t distance) {
    static constexpr uint32_t scale = 10; //1:X Meaning 1 cell is X mm
    //static constexpr uint32_t resolution = 10; //Jump X mm per iteration

    uint32_t scaled_distance = distance / scale;

    float angle_rad = angle * PI / 180.0;
    float dx = cos(angle_rad);
    float dy = sin(angle_rad);

    float ray_x = 0;
    float ray_y = 0;

    float ray_x_interpolate, ray_y_interpolate;
    volatile uint32_t absolute_x, absolute_y;

    vPortEnterCritical(&spinlock);
    uint32_t agent_pos_x = agent_x;
    uint32_t agent_pos_y = agent_y;
    vPortExitCritical(&spinlock);

    //Serial.println();

    for(uint32_t i = 0; i < scaled_distance; ++i) {
        ray_x += dx;
        ray_y += dy;

        ray_x_interpolate = round(ray_x);
        ray_y_interpolate = round(ray_y);

        absolute_x = static_cast<uint32_t>(agent_pos_x + ray_x_interpolate);
        absolute_y = static_cast<uint32_t>(agent_pos_y + ray_y_interpolate);

        // Serial.print(absolute_x);
        // Serial.print(" ");
        // Serial.print(absolute_y);
        // Serial.print(" | ");

        if((absolute_x >= 0) && (absolute_x < map_dim) && (absolute_y >= 0) && (absolute_y < map_dim)) {
            if(i == scaled_distance - 1) {
                vPortEnterCritical(&spinlock);
                map[absolute_x][absolute_y] = CellType::OBSTACLE;
                vPortExitCritical(&spinlock);
            }else{
                vPortEnterCritical(&spinlock);
                map[absolute_x][absolute_y] = CellType::EMPTY;
                vPortExitCritical(&spinlock);
            }
        }

    }
}

void WorldMap::process_lidar_frame(Lidar::lidarframe_t frame) {
    
    uint32_t step = (frame.end_angle - frame.start_angle)/(12-1);
    // Serial.print("step: ");
    // Serial.println(step);
    for(uint32_t i = 0; i < 12; ++i) {
        if(frame.data[i].intensity < 100) {
            continue;
        }

        uint32_t angle = frame.start_angle + step*i;
        uint32_t distance = frame.data[i].distance;
        raycast_data(angle, distance);
        // Serial.print("angle: ");
        // Serial.print(angle);
        // Serial.print(" distance: ");
        // Serial.println(distance);
    }
}

void WorldMap::tick_com() {
    //if(print_map) {
        if(millis() - chunk_print_timestamp > chunk_print_rate) {
            chunk_print_timestamp = millis();
            send_next_chunk();
            //print_all_ascii();
        }
    //}
}

void WorldMap::tick_frame() {

    Lidar::lidarframe_t frame = _lidar.get_lidar_frame();

    if(frame.header == 0x54) {
        digitalWrite(6, !digitalRead(6));
        process_lidar_frame(frame);
    }

}

