#ifndef WAREHOUSE_MANAGER_H
#define WAREHOUSE_MANAGER_H

#include "alley.h"
#include <vector>
#include <string>
#include <unordered_map>

class WarehouseManager {
public:
    // Constructor initializes warehouse with custom shelf statuses and predefined base routes
    WarehouseManager(const std::unordered_map<ItemType, std::vector<bool>>& initial_shelf_statuses);

    // Finds an empty shelf in the alley for a specific item type
    std::string store_item(ItemType item_type);

    // Empties a specific shelf in the alley for a specific item type
    bool retrieve_item(ItemType item_type, int shelf_index);

    // Gets a list of all empty shelves for a specified item type
    std::vector<std::string> get_empty_shelves_for_type(ItemType item_type) const;

    // Displays the status of all aisles and their shelves in the warehouse
    void display_warehouse_status() const;

    // Returns a route for the specified item type, adding an available shelf at the transit point
    std::vector<std::string> get_route_for_item(ItemType item_type, std::vector<std::string>& FBroute);

    std::vector<std::string> get_point_for_item(ItemType item_type);

private:
    std::vector<Alley> alleys;                           // Collection of alleys, each storing a specific item type
    std::unordered_map<ItemType, std::vector<std::string>> base_routes; // Base routes for each item type
    std::unordered_map<ItemType, std::vector<std::string>> base_points;

    // Helper to get a string name for the alley based on item type
    std::string get_alley_name(ItemType item_type) const;
};

#endif // WAREHOUSE_MANAGER_H


