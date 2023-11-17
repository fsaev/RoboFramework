
#ifndef PATH_H
#define PATH_H

#include "map.h"

struct Node
{
    uint32_t x;
    uint32_t y;
    uint32_t parent_x;
    uint32_t parent_y;
    float g_cost;
    float h_cost;
    float f_cost;
};

class Path {
public:
    // Constructor
    explicit Path(WorldMap& map);

    // Other member functions
    void set_from_to(std::pair<uint32_t, uint32_t> from, std::pair<uint32_t, uint32_t> to);
    bool path_found();

    void tick();

private:
    WorldMap& map;
    std::pair<uint32_t, uint32_t> from;
    std::pair<uint32_t, uint32_t> to;

    static constexpr uint32_t x_max = WorldMap::map_dim;
    static constexpr uint32_t x_step = 10;
    static constexpr uint32_t y_max = WorldMap::map_dim;
    static constexpr uint32_t y_step = 10;
    //std::array<std::array<WorldMap::CellType, WorldMap::map_dim>, WorldMap::map_dim> search_map;
    // Member variables

};

#endif // PATH_H
