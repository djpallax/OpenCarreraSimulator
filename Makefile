CFLAGS = -std=c++17 -O2 -I$(SRC_DIR)/external/tinyxml2
LDFLAGS = -lglfw -lvulkan -ldl -lpthread -lX11 -lXxf86vm -lXrandr -lXi #-ltinyxml2
SRC_DIR = src
OBJ_DIR = obj
TARGET = VulkanTest

SRC_FILES = $(SRC_DIR)/main/main.cpp $(SRC_DIR)/menu/Menu.cpp $(SRC_DIR)/xml/XMLHandler.cpp $(SRC_DIR)/external/tinyxml2/tinyxml2.cpp
OBJ_FILES = $(SRC_FILES:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)

all: $(TARGET)

$(TARGET): $(OBJ_FILES)
	g++ $(CFLAGS) -o $(TARGET) $(OBJ_FILES) $(LDFLAGS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	g++ $(CFLAGS) -c $< -o $@

.PHONY: test clean

test: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(OBJ_FILES) $(TARGET)
	rm -rf $(OBJ_DIR)
