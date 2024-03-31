#include <stdbool.h>
#include <stdint.h>

#define MAZE_SIZE 10 // Assuming a 10x10 maze for simplicity
#define WALL -1
#define UNVISITED 0xFFFF
#define GOAL 0

uint16_t maze[MAZE_SIZE][MAZE_SIZE];

// Function prototypes
void initializeMaze();
void floodFill(uint8_t x, uint8_t y, uint16_t value);
bool isValid(uint8_t x, uint8_t y);

void main() {
    initializeMaze();
    // Assuming the finish is at (9,9) for this example
    floodFill(9, 9, GOAL);
    // After calling floodFill, the maze array contains the steps from each cell to the finish.
}

void initializeMaze() {
    for (uint8_t x = 0; x < MAZE_SIZE; x++) {
        for (uint8_t y = 0; y < MAZE_SIZE; y++) {
            maze[x][y] = UNVISITED; // Initialize all cells as unvisited
        }
    }
    // Manually set walls based on your maze's layout
    // Example: maze[1][1] = WALL;
}

void floodFill(uint8_t x, uint8_t y, uint16_t value) {
    if (!isValid(x, y) || maze[x][y] <= value) {
        return; // Stop if out of bounds, is a wall, or already has a lower value
    }
    maze[x][y] = value;
    // Recursively fill neighbors, incrementing the value
    floodFill(x + 1, y, value + 1);
    floodFill(x - 1, y, value + 1);
    floodFill(x, y + 1, value + 1);
    floodFill(x, y - 1, value + 1);
}

bool isValid(uint8_t x, uint8_t y) {
    // Check if the coordinates are within the maze bounds and not a wall
    return x < MAZE_SIZE && y < MAZE_SIZE && maze[x][y] != WALL;
}
