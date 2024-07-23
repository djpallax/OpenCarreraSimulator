#include "XMLHandler.h"
#include <tinyxml2.h>
#include <iostream>

using namespace tinyxml2;

void XMLHandler::parseXML(const std::string& filename, std::vector<MenuOption>& options) {
    XMLDocument doc;
    if (doc.LoadFile(filename.c_str()) != XML_SUCCESS) {
        std::cerr << "Failed to load file: " << filename << "\n";
        return;
    }

    XMLElement* menuElement = doc.FirstChildElement("menu");
    if (!menuElement) {
        std::cerr << "No menu element found\n";
        return;
    }

    XMLElement* optionElement = menuElement->FirstChildElement("option");
    while (optionElement) {
        MenuOption option;
        option.name = optionElement->Attribute("name");
        option.action = optionElement->Attribute("action");
        options.push_back(option);
        optionElement = optionElement->NextSiblingElement("option");
    }
}
