/*
    * path.cpp
    *
    * Created on: 17. Nov 2023
    * 
    * Based on: https://dev.to/jansonsa/a-star-a-path-finding-c-4a4h
*/

#include "path.h"
#include "map.h"
#include <cmath>
#include <limits>
#include <stack>


Path::Path(WorldMap& map) : map(map) {
    // Constructor
}

bool Path::is_valid(std::pair<uint32_t, uint32_t> pos) { //If our Node is an obstacle it is not valid
    //uint32_t id = pos.first + pos.second * (x_max / x_step);
    if (map.get_cell(pos) == WorldMap::CellType::EMPTY) {
        if (pos.first < 0 || pos.second < 0 
            || pos.first >= (x_max / x_step) || pos.second >= (y_max / y_step)) {
            return false;
        }
        return true;
    } 
    return false;
}

bool Path::is_destination(std::pair<uint32_t, uint32_t> pos, Node dest) {
    if (pos.first == dest.x && pos.second == dest.y) {
        return true;
    }
    return false;
}

float Path::calculate_h(std::pair<uint32_t, uint32_t> pos, Node dest) {
    // Return using the distance formula
    return static_cast<float>(sqrt((pos.first - dest.x) * (pos.first - dest.x)
        + (pos.second - dest.y) * (pos.second - dest.y)));
}

std::vector<Node> Path::a_star(Node robot, Node target) {
    std::vector<Node> empty;
    if(is_valid(std::make_pair(target.x, target.y)) == false) {
        //Destination is invalid
        return empty;
    }

    if(is_destination(std::make_pair(robot.x, robot.y), target) == true) {
        //You clicked on yourself
        return empty;
    }

    std::array<std::array<bool, y_max / y_step>, x_max / x_step> closed_list;

    //Initialize whole map
    for (int x = 0; x < (x_max / x_step); x++) {
        for (int y = 0; y < (y_max / y_step); y++) {
            node_map[x][y].f_cost = std::numeric_limits<float>::max();
            node_map[x][y].g_cost = std::numeric_limits<float>::max();
            node_map[x][y].h_cost = std::numeric_limits<float>::max();
            node_map[x][y].parent_x = -1;
            node_map[x][y].parent_y = -1;
            node_map[x][y].x = x;
            node_map[x][y].y = y;

            closed_list[x][y] = false;
        }
    }

    //Initialize our starting list
    uint32_t x = robot.x;
    uint32_t y = robot.y;
    node_map[x][y].f_cost = 0.0;
    node_map[x][y].g_cost = 0.0;
    node_map[x][y].h_cost = 0.0;
    node_map[x][y].parent_x = x;
    node_map[x][y].parent_y = y;

    std::vector<Node> open_list;
    open_list.emplace_back(node_map[x][y]);
    bool destination_found = false;

    while(!open_list.empty() && open_list.size() < (x_max / x_step) * (y_max / y_step)) {
        Node node;
        do {
            float temp = std::numeric_limits<float>::max();
            std::vector<Node>::iterator itNode;
            for (std::vector<Node>::iterator it = open_list.begin();
                it != open_list.end(); it = std::next(it)) {
                Node n = *it;
                if (n.f_cost < temp) {
                    temp = n.f_cost;
                    itNode = it;
                }
            }
            node = *itNode;
            open_list.erase(itNode);
        } while (is_valid(std::make_pair(node.x, node.y)) == false);

        x = node.x;
        y = node.y;
        closed_list[x][y] = true;

        //For each neighbour starting from North-West to South-East
        for (uint32_t new_x = -1; new_x <= 1; new_x++) {
            for (uint32_t new_y = -1; new_y <= 1; new_y++) {
                float g_new, h_new, f_new;
                if (is_valid(std::make_pair(x + new_x, y + new_y))) {
                    if (is_destination(std::make_pair(x + new_x, y + new_y), target))
                    {
                        //Destination found - make path
                        node_map[x + new_x][y + new_y].parent_x = x;
                        node_map[x + new_x][y + new_y].parent_y = y;
                        destination_found = true;
                        return make_path(node_map, target);
                    }
                    else if (closed_list[x + new_x][y + new_y] == false)
                    {
                        g_new = node.g_cost + 1.0;
                        h_new = calculate_h(std::make_pair(x + new_x, y + new_y), target);
                        f_new = g_new + h_new;
                        // Check if this path is better than the one already present
                        if (node_map[x + new_x][y + new_y].f_cost == std::numeric_limits<float>::max() ||
                            node_map[x + new_x][y + new_y].f_cost > f_new)
                        {
                            // Update the details of this neighbour node
                            node_map[x + new_x][y + new_y].f_cost = f_new;
                            node_map[x + new_x][y + new_y].g_cost = g_new;
                            node_map[x + new_x][y + new_y].h_cost = h_new;
                            node_map[x + new_x][y + new_y].parent_x = x;
                            node_map[x + new_x][y + new_y].parent_y = y;
                            open_list.emplace_back(node_map[x + new_x][y + new_y]);
                        }
                    }
                }
            }
        }
    }
}

std::vector<Node> Path::make_path(std::array<std::array<Node, (y_max / y_step)>, (x_max / x_step)> map, Node dest) {
    uint32_t x = dest.x;
    uint32_t y = dest.y;
    std::stack<Node> path;
    std::vector<Node> usable_path;

    while (!(map[x][y].parent_x == x && map[x][y].parent_y == y)
        && map[x][y].x != -1 && map[x][y].y != -1) 
    {
        path.push(map[x][y]);
        uint32_t temp_x = map[x][y].parent_x;
        uint32_t temp_y = map[x][y].parent_y;
        x = temp_x;
        y = temp_y;

    }
    path.push(map[x][y]);

    while (!path.empty()) {
        Node top = path.top();
        path.pop();
        usable_path.emplace_back(top);
    }
    return usable_path;
}