#ifndef MAP_H
#define MAP_H

#include <cstdint>
#include <array>
#include "lidar.h"

class WorldMap {
public:
    enum class CellType : uint8_t {
        EMPTY = 0,
        OBSTACLE = 1,
        ROBOT = 2,
        TARGET = 3,
        
        //Path variant for visualization
        PATH_EMPTY = 4,
        PATH_OBSTACLE = 5, //Invalid
        PATH_ROBOT = 6,
        PATH_TARGET = 7,
    };

    static constexpr uint32_t map_dim = 140;

    explicit WorldMap(Lidar& lidar);
    void example_fill(std::pair<uint32_t, uint32_t> top_left);
    void print_all_ascii();
    void enable_printing();
    void disable_printing();
    CellType get_cell(std::pair<uint32_t, uint32_t> pos);
    std::pair<uint32_t, uint32_t> get_agent_pos();
    void tick_com();
    void tick_frame(); //Thread safe (i think)
    bool save();

private:
    // Init agent in the middle of the map
    uint32_t agent_x = map_dim/2;
    uint32_t agent_y = map_dim/2;

    Lidar& _lidar;

    static constexpr uint32_t chunk_print_rate = 5; //Full refresh: ((map_dim*map_dim)/chunk_size) * 100 ms)
    uint32_t chunk_print_timestamp = 0;

    static constexpr uint32_t chunk_size = 140;
    uint32_t chunk_printidx = 0;
    bool print_map = false;

    static constexpr std::array<std::array<CellType, 8>, 8> example_map = {{
        {CellType::EMPTY, CellType::EMPTY, CellType::EMPTY, CellType::OBSTACLE, CellType::OBSTACLE, CellType::EMPTY, CellType::EMPTY, CellType::EMPTY},
        {CellType::EMPTY, CellType::EMPTY, CellType::OBSTACLE, CellType::EMPTY, CellType::EMPTY, CellType::OBSTACLE, CellType::EMPTY, CellType::EMPTY},
        {CellType::EMPTY, CellType::OBSTACLE, CellType::EMPTY, CellType::EMPTY, CellType::EMPTY, CellType::OBSTACLE, CellType::EMPTY, CellType::EMPTY},
        {CellType::EMPTY, CellType::OBSTACLE, CellType::EMPTY, CellType::EMPTY, CellType::EMPTY, CellType::EMPTY, CellType::OBSTACLE, CellType::EMPTY},
        {CellType::EMPTY, CellType::OBSTACLE, CellType::EMPTY, CellType::EMPTY, CellType::EMPTY, CellType::EMPTY, CellType::OBSTACLE, CellType::EMPTY},
        {CellType::EMPTY, CellType::EMPTY, CellType::OBSTACLE, CellType::EMPTY, CellType::EMPTY, CellType::EMPTY, CellType::OBSTACLE, CellType::EMPTY},
        {CellType::EMPTY, CellType::EMPTY, CellType::EMPTY, CellType::OBSTACLE, CellType::EMPTY, CellType::OBSTACLE, CellType::EMPTY, CellType::EMPTY},
        {CellType::EMPTY, CellType::EMPTY, CellType::EMPTY, CellType::EMPTY, CellType::OBSTACLE, CellType::EMPTY, CellType::EMPTY, CellType::EMPTY},
    }};

    std::array<std::array<CellType, map_dim>, map_dim> map;

    void raycast_data(uint32_t angle, uint32_t distance);
    void process_lidar_frame(Lidar::lidarframe_t frame);
    void send_next_chunk();
};

#endif // MAP_H
