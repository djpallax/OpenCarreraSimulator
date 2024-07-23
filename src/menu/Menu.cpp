#include "Menu.h"
#include "../xml/XMLHandler.h"
#include <iostream>

void Menu::loadFromFile(const std::string& filename) {
    XMLHandler xmlHandler;
    xmlHandler.parseXML(filename, options);
}

void Menu::display() const {
    std::cout << "Menu:\n";
    for (const auto& option : options) {
        std::cout << option.name << "\n";
    }
}
