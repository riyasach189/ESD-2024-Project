#include <stdint.h>

// Directions
typedef enum {
    NORTH,
    EAST,
    SOUTH,
    WEST
} Direction;

// Position
typedef struct {
    int x;
    int y;
    Direction dir;
} Position;

// Function to update the mouse's position
void updatePosition(Position* position, int encoderLeft, int encoderRight) {
    // Assuming a simple movement system where each cell move results in a fixed encoder count increase
    // This function would be called after a movement is completed, and the encoders are reset

    // Placeholder for encoder count threshold to consider as a cell move
    const int encoderThreshold = 100; // Example value, adjust based on your calibration
    
    // Check if we have moved a cell
    if(encoderLeft > encoderThreshold && encoderRight > encoderThreshold) {
        switch(position->dir) {
            case NORTH:
                position->y++;
                break;
            case SOUTH:
                position->y--;
                break;
            case EAST:
                position->x++;
                break;
            case WEST:
                position->x--;
                break;
        }
        
        // Reset encoder counts after moving one cell
        // This might actually be done elsewhere in your code, after motors are stopped
        //encoderLeft = 0;
        //encoderRight = 0;
    }

    // Optionally, adjust direction based on encoder difference for turning
    // This part is simplified; turning involves changing the direction and is likely controlled by a different part of your system
}

// Example usage
int main() {
    Position mousePosition = {0, 0, NORTH}; // Starting at (0, 0) facing North

    // Example move: suppose we have moved one cell north
    int encoderLeft = 120; // Example encoder values after moving
    int encoderRight = 120;

    updatePosition(&mousePosition, encoderLeft, encoderRight);

    // Print new position
    printf("New Position: (%d, %d) facing %d\n", mousePosition.x, mousePosition.y, mousePosition.dir);

    return 0;
}
