
#ifndef BLE_H
#define BLE_H

#include <string>
#include "map.h"

class BLE {
public:
    BLE(const std::string name, WorldMap* map) : _name(name), _map(map) {};  // Constructor
    ~BLE() {}; // Destructor

    void start_server();

    // Add your class methods here

private:
    std::string _name;
    WorldMap* _map;
    // Add your private members here

    
};

#endif // BLE_H
