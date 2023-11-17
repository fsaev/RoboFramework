/*
    * path.cpp
    *
    * Created on: 17. Nov 2023
    * 
    * Based on: https://dev.to/jansonsa/a-star-a-path-finding-c-4a4h
*/

#include "path.h"
#include "map.h"

Path::Path(WorldMap& map) : map(map) {
    // Constructor
}

bool Path::isValid(int x, int y) { //If our Node is an obstacle it is not valid
    int id = x + y * (X_MAX / X_STEP);
    if (world.obstacles.count(id) == 0) {
        if (x < 0 || y < 0 || x >= (X_MAX / X_STEP) || y >= (Y_MAX / Y_STEP)) {
            return false;
        }
        return true;
    } 
    return false;
}

