#include "warehouse_manager.h"
#include <iostream>

// Constructor initializes warehouse with aisles and predefined base routes for each item type
WarehouseManager::WarehouseManager(const std::unordered_map<ItemType, std::vector<bool>>& initial_shelf_statuses) {
    // Initialize alleys with given initial shelf statuses
    for (const auto& [item_type, shelf_status] : initial_shelf_statuses) {
        std::string alley_name = get_alley_name(item_type);
        alleys.emplace_back(alley_name, item_type, shelf_status);
    }

    // Define base routes for each item type
    base_routes[ItemType::RAW] = {"entry_shelf_raw", "transit_in_shelf_4", "Delivering_3", "transit_out_shelf_4"};
    base_routes[ItemType::HALF] = {"entry_shelf_half", "transit_in_shelf_5", "Delivering_2", "transit_out_shelf_5"};
    base_routes[ItemType::FINISHED] = {"entry_shelf_finished", "transit_in_shelf_2", "Delivering_1", "transit_out_shelf_2"};

    base_points[ItemType::RAW] = {"pickup_1", "entry_shelf_raw"};
    base_points[ItemType::HALF] = {"pickup_2", "entry_shelf_half"};
    base_points[ItemType::FINISHED] = {"pickup_3", "entry_shelf_finished"};
}

std::vector<std::string> WarehouseManager::get_point_for_item(ItemType item_type) {
    auto points = base_points[item_type];
    return points;
}

// Gets the base route for the item type, and then adds the transit shelf location based on availability
std::vector<std::string> WarehouseManager::get_route_for_item(ItemType item_type, std::vector<std::string>& FBroute) {
    auto route = base_routes[item_type];
    FBroute.clear(); 

    // Determine the correct transit aisle for this item type
    for (auto& alley : alleys) {
        if (alley.get_item_type() == item_type) {
            // Find an empty shelf in the alley at the transit point
            int empty_shelf_index = alley.find_empty_shelf();
            if (empty_shelf_index != -1) {
                // Add the empty shelf location to the route
                std::string shelf_location = alley.get_name() + "_shelf_" + std::to_string(empty_shelf_index);
                std::string shelf_location_fb = alley.get_name() + "_goal_" + std::to_string(empty_shelf_index);
                route.push_back(shelf_location);
                FBroute.push_back(shelf_location_fb);
                FBroute.push_back(shelf_location);
                bool update_success = alley.set_shelf_status(empty_shelf_index, true);
                break;
            } else {
                std::cerr << "No empty shelves available in " << alley.get_name() << " for item type: " << static_cast<int>(item_type) << std::endl;
            }
        }
    }

    return route;
}

void WarehouseManager::display_warehouse_status() const {
    std::cout << "Warehouse Status:" << std::endl;
    for (const auto& alley : alleys) {
        alley.display_shelves(); // Display shelves in each alley
    }
}

// Returns a string representing the alley name based on the item type
std::string WarehouseManager::get_alley_name(ItemType item_type) const {
    switch (item_type) {
        case ItemType::RAW: return "Alley_RAW";
        case ItemType::HALF: return "Alley_HALF";
        case ItemType::FINISHED: return "Alley_FINISHED";
        default: return "Alley_UNKNOWN";
    }
}

// Remaining methods (store_item, retrieve_item, display_warehouse_status) can stay the same
