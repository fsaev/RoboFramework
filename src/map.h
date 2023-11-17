#ifndef MAP_H
#define MAP_H

#include <cstdint>
#include <array>

class WorldMap {
public:
    enum class CellType : uint8_t {
        EMPTY = 0,
        OBSTACLE = 1,
        ROBOT = 2,
        TARGET = 3
    };

    static constexpr uint32_t map_dim = 192;

    explicit WorldMap() {};
    void example_fill(std::pair<uint32_t, uint32_t> top_left);
    void print_all_ascii();
    void enable_printing();
    void disable_printing();
    void tick();
    bool save();

private:
    static constexpr uint32_t chunk_print_rate = 10; //Full refresh: ((map_dim*map_dim)/chunk_size) * 100 ms)
    uint32_t chunk_print_timestamp = 0;

    static constexpr uint32_t chunk_size = 32;
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

    void send_next_chunk();
};

#endif // MAP_H
