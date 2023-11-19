
#ifndef PATH_H
#define PATH_H

#include <vector>
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

inline bool operator < (const Node& lhs, const Node& rhs)
{//We need to overload "<" to put our struct into a set
	return lhs.f_cost < rhs.f_cost;
}

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
    static constexpr uint32_t x_step = 4;
    static constexpr uint32_t y_max = WorldMap::map_dim;
    static constexpr uint32_t y_step = 4;

    std::array<std::array<Node, y_max / y_step>, x_max / x_step> node_map; //Scaled node map
    //std::array<std::array<WorldMap::CellType, WorldMap::map_dim>, WorldMap::map_dim> search_map;
    // Member variables

    bool is_valid(std::pair<uint32_t, uint32_t> pos);
    bool is_destination(std::pair<uint32_t, uint32_t> pos, Node dest);

    float calculate_h(std::pair<uint32_t, uint32_t> pos, Node dest);

    std::vector<Node> a_star(Node robot, Node target);
    std::vector<Node> make_path(std::array<std::array<Node, (y_max / y_step)>, (x_max / x_step)> map, Node dest);

};

#endif // PATH_H
