#include "alley.h"

// Constructor to initialize an alley with a specific item type and custom shelf configuration
Alley::Alley(const std::string& name, ItemType item_type, const std::vector<bool>& initial_shelf_status)
    : name(name), item_type(item_type), shelves(initial_shelf_status) {}

// Finds the first available shelf (returns index or -1 if none are available)
int Alley::find_empty_shelf() const {
    for (size_t i = 0; i < shelves.size(); i++) {  // Changed `int` to `size_t` for `i`
        if (!shelves[i]) { // Check if the shelf is empty
            return static_cast<int>(i);  // Cast to `int` to match return type
        }
    }
    return -1; // No empty shelves available
}

// Mark a specific shelf as occupied or empty
bool Alley::set_shelf_status(int index, bool is_full) {
    if (index >= 0 && static_cast<size_t>(index) < shelves.size()) {  // Cast `index` to `size_t`
        shelves[index] = is_full;
        return true;
    }
    return false;
}

// Returns a list of all empty shelf indices in this alley
std::vector<int> Alley::get_empty_shelves() const {
    std::vector<int> empty_shelves;
    for (size_t i = 0; i < shelves.size(); i++) {  // Changed `int` to `size_t` for `i`
        if (!shelves[i]) { // Check if shelf is empty
            empty_shelves.push_back(static_cast<int>(i));  // Cast `i` to `int`
        }
    }
    return empty_shelves;
}

// Displays the status of all shelves in this alley
void Alley::display_shelves() const {
    std::cout << "Alley: " << name << " (ItemType: " << static_cast<int>(item_type) << ")" << std::endl;
    for (size_t i = 0; i < shelves.size(); i++) {  // Changed `int` to `size_t` for `i`
        std::cout << "  Shelf " << i << ": " << (shelves[i] ? "Full" : "Empty") << std::endl;
    }
}

// Getters
ItemType Alley::get_item_type() const { return item_type; }
const std::string& Alley::get_name() const { return name; }
