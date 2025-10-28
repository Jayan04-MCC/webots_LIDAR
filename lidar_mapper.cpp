/*
 * LIDAR Mapper Controller for Webots
 * Processes LIDAR data to generate 2D maps and detect obstacles in real-time
 *
 * Author: Claude Code
 * Date: 2025-10-27
 */

#include <webots/Robot.hpp>
#include <webots/Lidar.hpp>
#include <webots/Display.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

using namespace webots;
using namespace std;

// Constants
const int TIME_STEP = 64;           // Simulation timestep in ms
const double MAP_SIZE = 10.0;       // Map size in meters (10x10m)
const int GRID_SIZE = 200;          // Grid resolution (200x200 cells)
const double CELL_SIZE = MAP_SIZE / GRID_SIZE;  // Size of each cell in meters
const double OBSTACLE_THRESHOLD = 0.6;  // Probability threshold for obstacle detection
const double FREE_THRESHOLD = 0.4;      // Probability threshold for free space

// Obstacle structure
struct Obstacle {
    double x;           // X position in meters
    double y;           // Y position in meters
    double distance;    // Distance from robot
    double angle;       // Angle from robot
};

/*
 * OccupancyGrid Class
 * Implements a 2D occupancy grid for mapping
 */
class OccupancyGrid {
private:
    vector<vector<double>> grid;  // Probability values [0,1]
    int size;
    double cellSize;
    double mapSize;

public:
    OccupancyGrid(int gridSize, double mapSizeMeters)
        : size(gridSize), mapSize(mapSizeMeters) {
        cellSize = mapSize / size;
        grid.resize(size, vector<double>(size, 0.5)); // Initialize with unknown (0.5)
    }

    // Convert world coordinates to grid indices
    bool worldToGrid(double x, double y, int &gridX, int &gridY) {
        gridX = static_cast<int>((x + mapSize/2.0) / cellSize);
        gridY = static_cast<int>((y + mapSize/2.0) / cellSize);

        return (gridX >= 0 && gridX < size && gridY >= 0 && gridY < size);
    }

    // Update cell with new measurement (Bayesian update)
    void updateCell(int x, int y, bool occupied) {
        if (x < 0 || x >= size || y < 0 || y >= size) return;

        double prior = grid[x][y];
        double likelihood = occupied ? 0.9 : 0.1;

        // Bayesian update
        double posterior = (likelihood * prior) /
                          (likelihood * prior + (1 - likelihood) * (1 - prior));

        grid[x][y] = posterior;
    }

    // Get probability value at cell
    double getCell(int x, int y) const {
        if (x < 0 || x >= size || y < 0 || y >= size) return 0.5;
        return grid[x][y];
    }

    int getSize() const { return size; }

    // Clear the grid
    void clear() {
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                grid[i][j] = 0.5;
            }
        }
    }
};

/*
 * LidarProcessor Class
 * Processes LIDAR data and updates the occupancy grid
 */
class LidarProcessor {
private:
    Lidar *lidar;
    OccupancyGrid *grid;
    double maxRange;
    int numPoints;

public:
    LidarProcessor(Lidar *l, OccupancyGrid *g) : lidar(l), grid(g) {
        maxRange = lidar->getMaxRange();
        numPoints = lidar->getHorizontalResolution();
    }

    // Process LIDAR scan and update grid
    void processScan(double robotX, double robotY, double robotTheta) {
        const float *rangeImage = lidar->getRangeImage();
        if (!rangeImage) return;

        double angleStep = lidar->getFov() / numPoints;
        double startAngle = -lidar->getFov() / 2.0;

        for (int i = 0; i < numPoints; i++) {
            float range = rangeImage[i];

            // Skip invalid readings
            if (range >= maxRange || range <= 0.0f || std::isinf(range)) {
                continue;
            }

            double angle = startAngle + i * angleStep + robotTheta;

            // Calculate hit point in world coordinates
            double hitX = robotX + range * cos(angle);
            double hitY = robotY + range * sin(angle);

            // Update grid cells along the ray
            updateRay(robotX, robotY, hitX, hitY, range);
        }
    }

private:
    // Bresenham-like ray tracing to update grid cells
    void updateRay(double x0, double y0, double x1, double y1, float range) {
        int gx0, gy0, gx1, gy1;

        if (!grid->worldToGrid(x0, y0, gx0, gy0)) return;
        if (!grid->worldToGrid(x1, y1, gx1, gy1)) {
            // Clamp endpoint to grid boundaries
            double dx = x1 - x0;
            double dy = y1 - y0;
            double t = 0.9; // Scale factor to keep within bounds
            x1 = x0 + dx * t;
            y1 = y0 + dy * t;
            if (!grid->worldToGrid(x1, y1, gx1, gy1)) return;
        }

        // Bresenham's line algorithm
        int dx = abs(gx1 - gx0);
        int dy = abs(gy1 - gy0);
        int sx = (gx0 < gx1) ? 1 : -1;
        int sy = (gy0 < gy1) ? 1 : -1;
        int err = dx - dy;

        int x = gx0;
        int y = gy0;

        while (true) {
            // Mark cells along ray as free (except last cell)
            if (x != gx1 || y != gy1) {
                grid->updateCell(x, y, false);
            } else {
                // Mark endpoint as occupied (obstacle)
                if (range < maxRange - 0.1) {
                    grid->updateCell(x, y, true);
                }
                break;
            }

            if (x == gx1 && y == gy1) break;

            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x += sx;
            }
            if (e2 < dx) {
                err += dx;
                y += sy;
            }
        }
    }
};

/*
 * ObstacleDetector Class
 * Detects obstacles from LIDAR data
 */
class ObstacleDetector {
private:
    double minDistance;
    double detectionAngle;

public:
    ObstacleDetector(double minDist = 0.5, double angle = M_PI)
        : minDistance(minDist), detectionAngle(angle) {}

    // Detect obstacles from LIDAR scan
    vector<Obstacle> detectObstacles(Lidar *lidar) {
        vector<Obstacle> obstacles;
        const float *rangeImage = lidar->getRangeImage();
        if (!rangeImage) return obstacles;

        int numPoints = lidar->getHorizontalResolution();
        double maxRange = lidar->getMaxRange();
        double angleStep = lidar->getFov() / numPoints;
        double startAngle = -lidar->getFov() / 2.0;

        for (int i = 0; i < numPoints; i++) {
            float range = rangeImage[i];

            // Check if reading is valid and within detection range
            if (range < maxRange && range > 0.0f && !std::isinf(range)) {
                double angle = startAngle + i * angleStep;

                // Only detect obstacles in front sector
                if (fabs(angle) <= detectionAngle / 2.0) {
                    Obstacle obs;
                    obs.distance = range;
                    obs.angle = angle;
                    obs.x = range * cos(angle);
                    obs.y = range * sin(angle);

                    obstacles.push_back(obs);
                }
            }
        }

        return obstacles;
    }

    // Find closest obstacle
    Obstacle* findClosest(vector<Obstacle> &obstacles) {
        if (obstacles.empty()) return nullptr;

        Obstacle *closest = &obstacles[0];
        for (size_t i = 1; i < obstacles.size(); i++) {
            if (obstacles[i].distance < closest->distance) {
                closest = &obstacles[i];
            }
        }

        return closest;
    }
};

/*
 * DisplayRenderer Class
 * Renders the map and obstacles on Webots Display
 */
class DisplayRenderer {
private:
    Display *display;
    int width;
    int height;

public:
    DisplayRenderer(Display *disp) : display(disp) {
        width = display->getWidth();
        height = display->getHeight();
    }

    // Render the occupancy grid
    void renderGrid(const OccupancyGrid &grid) {
        int gridSize = grid.getSize();
        double scaleX = (double)width / gridSize;
        double scaleY = (double)height / gridSize;

        for (int i = 0; i < gridSize; i++) {
            for (int j = 0; j < gridSize; j++) {
                double prob = grid.getCell(i, j);

                // Color mapping: 0.5=gray (unknown), 0=white (free), 1=black (occupied)
                int gray = static_cast<int>(255 * (1.0 - prob));
                int color = (gray << 16) | (gray << 8) | gray;

                // Draw cell
                int x = static_cast<int>(i * scaleX);
                int y = static_cast<int>(j * scaleY);
                int w = static_cast<int>(scaleX) + 1;
                int h = static_cast<int>(scaleY) + 1;

                display->setColor(color);
                display->fillRectangle(x, y, w, h);
            }
        }
    }

    // Draw robot position
    void drawRobot(double x, double y, double theta) {
        int gridSize = GRID_SIZE;
        double scaleX = (double)width / gridSize;
        double scaleY = (double)height / gridSize;

        // Convert world to grid coordinates
        int gridX = static_cast<int>((x + MAP_SIZE/2.0) / CELL_SIZE);
        int gridY = static_cast<int>((y + MAP_SIZE/2.0) / CELL_SIZE);

        int px = static_cast<int>(gridX * scaleX);
        int py = static_cast<int>(gridY * scaleY);

        // Draw robot as blue circle
        display->setColor(0x0000FF);
        display->fillOval(px - 3, py - 3, 6, 6);

        // Draw orientation line
        int lineLength = 8;
        int endX = px + static_cast<int>(lineLength * cos(theta));
        int endY = py + static_cast<int>(lineLength * sin(theta));
        display->drawLine(px, py, endX, endY);
    }

    // Draw obstacles
    void drawObstacles(const vector<Obstacle> &obstacles, double robotX, double robotY) {
        display->setColor(0xFF0000); // Red color

        int gridSize = GRID_SIZE;
        double scaleX = (double)width / gridSize;
        double scaleY = (double)height / gridSize;

        for (const auto &obs : obstacles) {
            // Calculate world position of obstacle
            double worldX = robotX + obs.x;
            double worldY = robotY + obs.y;

            // Convert to grid coordinates
            int gridX = static_cast<int>((worldX + MAP_SIZE/2.0) / CELL_SIZE);
            int gridY = static_cast<int>((worldY + MAP_SIZE/2.0) / CELL_SIZE);

            int px = static_cast<int>(gridX * scaleX);
            int py = static_cast<int>(gridY * scaleY);

            // Draw obstacle as red dot
            display->fillOval(px - 2, py - 2, 4, 4);
        }
    }

    // Draw text overlay
    void drawText(const string &text, int x, int y) {
        display->setColor(0x000000);
        display->drawText(text, x, y);
    }

    // Clear display
    void clear() {
        display->setColor(0xFFFFFF);
        display->fillRectangle(0, 0, width, height);
    }
};

/*
 * Main Controller
 */
int main(int argc, char **argv) {
    // Create robot instance
    Robot *robot = new Robot();

    // Get devices
    Lidar *lidar = robot->getLidar("lidar");
    Display *display = robot->getDisplay("display");
    Motor *leftMotor = robot->getMotor("left wheel motor");
    Motor *rightMotor = robot->getMotor("right wheel motor");
    Keyboard *keyboard = robot->getKeyboard();

    // Initialize devices
    if (!lidar) {
        cerr << "ERROR: Lidar device 'lidar' not found!" << endl;
        return 1;
    }

    if (!display) {
        cerr << "ERROR: Display device 'display' not found!" << endl;
        return 1;
    }

    lidar->enable(TIME_STEP);
    lidar->enablePointCloud();
    keyboard->enable(TIME_STEP);

    // Set motors to velocity control
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);

    // Create system components
    OccupancyGrid grid(GRID_SIZE, MAP_SIZE);
    LidarProcessor lidarProc(lidar, &grid);
    ObstacleDetector obstacleDetector(0.3, M_PI);
    DisplayRenderer renderer(display);

    // Robot state
    double robotX = 0.0;
    double robotY = 0.0;
    double robotTheta = 0.0;
    double leftSpeed = 0.0;
    double rightSpeed = 0.0;

    cout << "=== LIDAR Mapper Controller ===" << endl;
    cout << "Controls:" << endl;
    cout << "  Arrow Up/Down: Forward/Backward" << endl;
    cout << "  Arrow Left/Right: Turn left/right" << endl;
    cout << "  Space: Stop" << endl;
    cout << "  R: Reset map" << endl;
    cout << "================================" << endl;

    int frameCount = 0;

    // Main control loop
    while (robot->step(TIME_STEP) != -1) {
        frameCount++;

        // Handle keyboard input
        int key = keyboard->getKey();
        const double MAX_SPEED = 3.0;
        const double TURN_SPEED = 1.5;

        switch(key) {
            case Keyboard::UP:
                leftSpeed = MAX_SPEED;
                rightSpeed = MAX_SPEED;
                break;
            case Keyboard::DOWN:
                leftSpeed = -MAX_SPEED;
                rightSpeed = -MAX_SPEED;
                break;
            case Keyboard::LEFT:
                leftSpeed = -TURN_SPEED;
                rightSpeed = TURN_SPEED;
                break;
            case Keyboard::RIGHT:
                leftSpeed = TURN_SPEED;
                rightSpeed = -TURN_SPEED;
                break;
            case ' ':
                leftSpeed = 0.0;
                rightSpeed = 0.0;
                break;
            case 'R':
                grid.clear();
                cout << "Map reset!" << endl;
                break;
        }

        leftMotor->setVelocity(leftSpeed);
        rightMotor->setVelocity(rightSpeed);

        // Update robot position (simple odometry)
        double dt = TIME_STEP / 1000.0;
        double wheelRadius = 0.02;
        double axleLength = 0.052;

        double vLeft = leftSpeed * wheelRadius;
        double vRight = rightSpeed * wheelRadius;
        double v = (vLeft + vRight) / 2.0;
        double omega = (vRight - vLeft) / axleLength;

        robotX += v * cos(robotTheta) * dt;
        robotY += v * sin(robotTheta) * dt;
        robotTheta += omega * dt;

        // Normalize angle
        while (robotTheta > M_PI) robotTheta -= 2 * M_PI;
        while (robotTheta < -M_PI) robotTheta += 2 * M_PI;

        // Process LIDAR data
        lidarProc.processScan(robotX, robotY, robotTheta);

        // Detect obstacles
        vector<Obstacle> obstacles = obstacleDetector.detectObstacles(lidar);

        // Render visualization (update every 5 frames for performance)
        if (frameCount % 5 == 0) {
            renderer.clear();
            renderer.renderGrid(grid);
            renderer.drawRobot(robotX, robotY, robotTheta);
            renderer.drawObstacles(obstacles, robotX, robotY);

            // Draw status text
            char statusText[100];
            sprintf(statusText, "Obstacles: %zu | Pos: (%.2f, %.2f)",
                    obstacles.size(), robotX, robotY);
            renderer.drawText(statusText, 5, 5);

            // Print closest obstacle info
            Obstacle *closest = obstacleDetector.findClosest(obstacles);
            if (closest) {
                if (frameCount % 50 == 0) { // Print every ~3 seconds
                    cout << "Closest obstacle: "
                         << "dist=" << closest->distance << "m, "
                         << "angle=" << (closest->angle * 180.0 / M_PI) << "deg"
                         << endl;
                }
            }
        }
    }

    delete robot;
    return 0;
}
