#pragma once
#include <string>
#include <vector>

struct MenuOption {
    std::string name;
    std::string action;
};

class Menu {
public:
    void loadFromFile(const std::string& filename);
    void display() const;

private:
    std::vector<MenuOption> options;
};
