#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <array>
#include <utility>

enum class ItemTypes {
    RAW,
    HALF,
    FINISHED
};

struct Alley {
    // Alley name
    std::string name;

    // Number of shelves in each row
    int shelves_per_row;

    // Two rows in the alley, each consisting of pairs of <bool: is the shelf present, ItemTypes: type of item>
    std::vector<std::pair<bool, ItemTypes>> row1; // bool: whether the shelf is present, ItemTypes: type of item on the shelf
    std::vector<std::pair<bool, ItemTypes>> row2;

    // Alley constructor that initializes the rows with a specified number of shelves and item types for each row
    Alley(std::string name, int shelves_per_row, ItemTypes row1_item_type, ItemTypes row2_item_type) 
        : name(name), shelves_per_row(shelves_per_row) 
    {
        // Initialize rows with the given number of shelves, no shelves present initially
        for (int i = 0; i < shelves_per_row; i++) {
            row1.push_back({false, row1_item_type});  // false means no shelf is present
            row2.push_back({false, row2_item_type});
        }
    }

    // Method to add a shelf at a specific position in a row
    void add_shelf(int row, int index, ItemTypes item_type) {
        if (row == 1 && index >= 0 && index < row1.size()) {
            row1[index] = {true, item_type};  // Add a shelf at the specified index in row 1
        } else if (row == 2 && index >= 0 && index < row2.size()) {
            row2[index] = {true, item_type};  // Add a shelf at the specified index in row 2
        } else {
            std::cerr << "Invalid row or index for adding a shelf" << std::endl;
        }
    }

    // Method to remove a shelf at a specific position in a row
    void remove_shelf(int row, int index) {
        if (row == 1 && index >= 0 && index < row1.size()) {
            row1[index] = {false, row1[index].second};  // Remove the shelf (set boolean to false)
        } else if (row == 2 && index >= 0 && index < row2.size()) {
            row2[index] = {false, row2[index].second};  // Remove the shelf (set boolean to false)
        } else {
            std::cerr << "Invalid row or index for removing a shelf" << std::endl;
        }
    }

    // Method to check the status of shelves
    void display_shelves() {
        std::cout << "Alley: " << name << std::endl;
        
        std::cout << "Row 1: " << std::endl;
        for (int i = 0; i < row1.size(); i++) {
            std::cout << "Shelf " << i << ": Present = " << row1[i].first << ", ItemType = " << static_cast<int>(row1[i].second) << std::endl;
        }

        std::cout << "Row 2: " << std::endl;
        for (int i = 0; i < row2.size(); i++) {
            std::cout << "Shelf " << i << ": Present = " << row2[i].first << ", ItemType = " << static_cast<int>(row2[i].second) << std::endl;
        }
    }

    // Method to expand the number of shelves in each row (if needed in the future)
    void expand_shelves(int new_shelves_per_row) {
        if (new_shelves_per_row > shelves_per_row) {
            for (int i = shelves_per_row; i < new_shelves_per_row; i++) {
                row1.push_back({false, row1[0].second}); // Add empty shelf positions
                row2.push_back({false, row2[0].second});
            }
            shelves_per_row = new_shelves_per_row;
        }
    }
};


struct EntryPoint {
    // Define the entry point for three types of items (Raw, Half, Finished)
    std::array<std::pair<ItemTypes, bool>, 3> entry_shelves;

    // Storage capacity for backup shelves
    int storage_capacity;

    // Constructor
    EntryPoint(int storage_capacity) : storage_capacity(storage_capacity) {
        // Initialize entry_shelves for each item type (false means not full)
        entry_shelves = {{
            {ItemTypes::RAW, false},
            {ItemTypes::HALF, false},
            {ItemTypes::FINISHED, false}
        }};
    }

    // Method to update the status of a specific item type at the entry point
    void update_entry_shelf(ItemTypes type, bool is_full) {
        for (auto& shelf : entry_shelves) {
            if (shelf.first == type) {
                shelf.second = is_full;
                break;
            }
        }
    }

    // Method to display the current status of the entry point
    void display_entry_status() const {
        std::cout << "Entry Point Status:\n";
        for (const auto& shelf : entry_shelves) {
            std::string item_type_str;
            switch (shelf.first) {
                case ItemTypes::RAW: item_type_str = "RAW"; break;
                case ItemTypes::HALF: item_type_str = "HALF"; break;
                case ItemTypes::FINISHED: item_type_str = "FINISHED"; break;
            }
            std::cout << "  " << item_type_str << ": " << (shelf.second ? "Full" : "Empty") << "\n";
        }
        std::cout << "Storage capacity for backup: " << storage_capacity << "\n";
    }
};

struct TransitPoint {
    // Holds the current shelves waiting for transfer (4 spaces), each space can hold different item types
    std::vector<std::pair<bool, ItemTypes>> spaces;

    // TransitPoint constructor to initialize 4 empty spaces
    TransitPoint() {
        for (int i = 0; i < 4; i++) {
            spaces.push_back({false, ItemTypes::RAW}); // Initially, all spaces are empty with default type
        }
    }

    // Method to add a shelf with a specific item type to the transit point
    bool add_shelf_to_transit(ItemTypes item_type) {
        for (auto &space : spaces) {
            if (!space.first) { // Find the first available empty space
                space = {true, item_type}; // Mark the space as occupied and set the item type
                return true; // Shelf successfully added
            }
        }
        std::cerr << "Transit point is full! Cannot add more shelves." << std::endl;
        return false; // No available space
    }

    // Method to remove a shelf from a specific position in the transit point
    bool remove_shelf_from_transit(int index) {
        if (index >= 0 && index < spaces.size()) {
            if (spaces[index].first) { // Check if the space is occupied
                spaces[index] = {false, ItemTypes::RAW}; // Mark the space as empty, reset item type
                return true; // Shelf successfully removed
            } else {
                std::cerr << "No shelf at this position in the transit point." << std::endl;
            }
        } else {
            std::cerr << "Invalid index for the transit point." << std::endl;
        }
        return false;
    }

    // Method to check if there is an available space in the transit point
    bool has_available_space() const {
        for (const auto &space : spaces) {
            if (!space.first) {
                return true; // At least one empty space available
            }
        }
        return false; // All spaces are occupied
    }

    // Method to check the status of the transit point
    void display_transit_status() const {
        std::cout << "Transit Point Status:" << std::endl;
        for (int i = 0; i < spaces.size(); i++) {
            std::cout << "Space " << i + 1 << ": "
                      << (spaces[i].first ? "Occupied" : "Empty") 
                      << ", Item Type = " << static_cast<int>(spaces[i].second) 
                      << std::endl;
        }
    }
};

