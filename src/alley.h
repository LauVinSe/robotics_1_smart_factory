// #ifndef ALLEY_H
// #define ALLEY_H

// #include <string>
// #include <vector>
// #include <iostream>

// enum class ItemType {
//     RAW,
//     HALF,
//     FINISHED
// };

// class Alley {
// public:
//     /**
//      * Constructor to initialize an alley with a specific item type and custom shelf configuration.
//      * 
//      * @param name Name of the alley (e.g., "Alley_RAW").
//      * @param item_type Type of items stored in this alley (RAW, HALF, or FINISHED).
//      * @param initial_shelf_status A vector<bool> indicating initial shelf statuses (true = full, false = empty).
//      */
//     Alley(const std::string& name, ItemType item_type, const std::vector<bool>& initial_shelf_status);

//     /**
//      * Finds the first available (empty) shelf in this alley.
//      * 
//      * @return Index of the first empty shelf, or -1 if no shelves are available.
//      */
//     int find_empty_shelf() const;

//     /**
//      * Sets the status of a specific shelf.
//      * 
//      * @param index Index of the shelf to update.
//      * @param is_full True to mark the shelf as full, false to mark it as empty.
//      * @return True if the operation was successful, false if the index is out of range.
//      */
//     bool set_shelf_status(int index, bool is_full);

//     /**
//      * Returns a list of all empty shelf indices in this alley.
//      * 
//      * @return A vector<int> containing indices of all empty shelves.
//      */
//     std::vector<int> get_empty_shelves() const;

//     /**
//      * Displays the current status of all shelves in this alley.
//      */
//     void display_shelves() const;

//     /**
//      * Getter for the item type stored in this alley.
//      * 
//      * @return The item type (RAW, HALF, or FINISHED) of this alley.
//      */
//     ItemType get_item_type() const;

//     /**
//      * Getter for the name of this alley.
//      * 
//      * @return The name of the alley as a string.
//      */
//     const std::string& get_name() const;

// private:
//     std::string name;                 // Name of the alley (e.g., "Alley_RAW")
//     ItemType item_type;               // Type of items stored in this alley (RAW, HALF, or FINISHED)
//     std::vector<bool> shelves;        // Shelf status (true = full, false = empty)
// };

// #endif // ALLEY_H

#ifndef ALLEY_H
#define ALLEY_H

#include <string>
#include <vector>
#include <iostream>

enum class ItemType {
    RAW,
    HALF,
    FINISHED
};

class Alley {
public:
    // Constructor
    Alley(const std::string& name, ItemType item_type, const std::vector<bool>& initial_shelf_status);

    // Finds the first available shelf (returns index or -1 if none are available)
    int find_empty_shelf() const;

    // Mark a specific shelf as occupied or empty
    bool set_shelf_status(int index, bool is_full);

    // Returns a list of all empty shelf indices in this alley
    std::vector<int> get_empty_shelves() const;

    // Displays the status of all shelves in this alley
    void display_shelves() const;

    // Getters
    ItemType get_item_type() const;
    const std::string& get_name() const;

private:
    std::string name;
    ItemType item_type;
    std::vector<bool> shelves;  // Shelf status (true = full, false = empty)
};

#endif // ALLEY_H

