#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// Assume these are defined according to your maze dimensions.
#define MAX_WIDTH 100
#define MAX_HEIGHT 100

int maze[MAX_WIDTH][MAX_HEIGHT];
bool visited[MAX_WIDTH][MAX_HEIGHT] = {false};

// Directions: right, left, down, up
int dx[] = {1, -1, 0, 0};
int dy[] = {0, 0, 1, -1};

typedef struct {
    uint8_t x, y;
} Point;

// Queue for BFS
Point queue[MAX_WIDTH * MAX_HEIGHT];
int front = 0, rear = 0;

void enqueue(Point p) {
    queue[rear++] = p;
}

Point dequeue() {
    return queue[front++];
}

bool isEmpty() {
    return front == rear;
}

bool isValid(int x, int y) {
    return x >= 0 && x < MAX_WIDTH && y >= 0 && y < MAX_HEIGHT && maze[x][y] == 0 && !visited[x][y];
}

void solveMaze(uint8_t startX, uint8_t startY, uint8_t endX, uint8_t endY) {
    enqueue((Point){startX, startY});
    visited[startX][startY] = true;
    maze[startX][startY] = 1; // Starting point distance is 1 to differentiate from unvisited cells.

    while (!isEmpty()) {
        Point p = dequeue();
        // If the end point is reached
        if (p.x == endX && p.y == endY) {
            printf("Maze solved! Distance to end: %d\n", maze[p.x][p.y] - 1);
            return; // Optionally, continue to mark all reachable areas.
        }

        for (int i = 0; i < 4; i++) { // Explore all directions
            int newX = p.x + dx[i];
            int newY = p.y + dy[i];

            if (isValid(newX, newY)) {
                visited[newX][newY] = true;
                maze[newX][newY] = maze[p.x][p.y] + 1; // Increment distance
                enqueue((Point){newX, newY});
            }
        }
    }

    printf("Maze could not be solved.\n");
}
