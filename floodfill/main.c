#include <stdio.h>
#include <stdbool.h>
#include <math.h> // For fabs function

#define MAX_WIDTH 10
#define MAX_HEIGHT 10
int maze[MAX_WIDTH][MAX_HEIGHT] = {{0}}; // Example maze structure
bool visited[MAX_WIDTH][MAX_HEIGHT] = {{false}};

// Assuming a simplified direction system for movement
typedef enum { NORTH, EAST, SOUTH, WEST } Direction;
typedef struct { int x, y; } Point;
typedef struct { float kp, ki, kd, setpoint, integral, lastError, outputLimit; } PIDController;
PIDController pid; // Single PID controller for simplicity

// Prototypes for PID and movement (implementations to follow)
void PID_Init(PIDController *pid, float kp, float ki, float kd, float setpoint, float outputLimit);
float PID_Compute(PIDController *pid, float currentValue);

// Movement and maze exploration
void moveMouse(Point *currentPosition, Direction dir);
void floodFillSolveMaze(Point currentPosition, int goalX, int goalY);

void PID_Init(PIDController *pid, float kp, float ki, float kd, float setpoint, float outputLimit) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = setpoint;
    pid->outputLimit = outputLimit;
    pid->integral = pid->lastError = 0.0;
}

float PID_Compute(PIDController *pid, float currentValue) {
    float error = pid->setpoint - currentValue;
    pid->integral += error;
    float derivative = error - pid->lastError;
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    if (output > pid->outputLimit) output = pid->outputLimit;
    if (output < -pid->outputLimit) output = -pid->outputLimit;
    pid->lastError = error;
    return output;
}

void moveMouse(Point *currentPosition, Direction dir) {
    // Simulate motor control to move one cell in the specified direction
    float distanceMoved = 0.0; // Simulate reading from an encoder
    float motorSpeed = PID_Compute(&pid, distanceMoved); // Compute PID output
    printf("Moving with speed: %f\n", motorSpeed); // Simulate setting motor speed

    // Update position based on direction (assuming each move is to an adjacent cell)
    switch (dir) {
        case NORTH: currentPosition->y += 1; break;
        case SOUTH: currentPosition->y -= 1; break;
        case EAST: currentPosition->x += 1; break;
        case WEST: currentPosition->x -= 1; break;
    }
    printf("New Position: (%d, %d)\n", currentPosition->x, currentPosition->y);
}

void floodFillSolveMaze(Point currentPosition, int goalX, int goalY) {
    // Placeholder for flood fill logic to navigate from currentPosition to (goalX, goalY)
    // This would involve recursively or iteratively exploring the maze, marking visited cells, and choosing the next move
    // For simplicity, we're assuming direct movement towards the goal without obstacles

    while (currentPosition.x != goalX || currentPosition.y != goalY) {
        if (currentPosition.x < goalX) moveMouse(&currentPosition, EAST);
        else if (currentPosition.x > goalX) moveMouse(&currentPosition, WEST);
        if (currentPosition.y < goalY) moveMouse(&currentPosition, NORTH);
        else if (currentPosition.y > goalY) moveMouse(&currentPosition, SOUTH);
    }
}

