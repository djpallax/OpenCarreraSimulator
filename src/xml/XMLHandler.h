#pragma once
#include "../menu/Menu.h"
#include <string>
#include <vector>

class XMLHandler {
public:
    void parseXML(const std::string& filename, std::vector<MenuOption>& options);
};
