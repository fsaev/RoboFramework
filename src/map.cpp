#include "map.h"
#include <Arduino.h>
#include <string>
#include <iostream>
#include <sstream>
constexpr std::array<std::array<WorldMap::CellType, 8>, 8> WorldMap::example_map;

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
                    chunk << "0";
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

    Serial.print(chunk.str().c_str());
}

void WorldMap::tick() {
    if(print_map) {
        if(millis() - chunk_print_timestamp > chunk_print_rate) {
            chunk_print_timestamp = millis();
            send_next_chunk();
        }
    }
}