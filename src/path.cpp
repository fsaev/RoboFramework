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
}