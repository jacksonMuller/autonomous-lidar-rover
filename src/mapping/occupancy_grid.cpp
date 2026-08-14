#include "mapping/occupancy_grid.h"
#include <cmath>

OccupancyGrid::OccupancyGrid(int width, int height, float resolution): width_(width), height_(height), resolution_(resolution), originX_(-(width * resolution) / 2.0f), originY_(-(height * resolution) / 2.0f), cells_(width * height, CellState::Unknown) {}

int OccupancyGrid::getWidth() const {
    return width_;
} 

int OccupancyGrid::getHeight() const {
    return height_;
}

float OccupancyGrid::getResolution() const {
    return resolution_;
}

CellState OccupancyGrid::getCell(int x, int y) const {
    int index = y * width_ + x;
    return cells_[index];
}

void OccupancyGrid::setCell(int x, int y, CellState state) {
    int index = y * width_ + x; 
    cells_[index] = state;
}

bool OccupancyGrid::isInBounds(int x, int y) const {
    return (x >= 0 && x < width_ && y >= 0 && y < height_);
}

bool OccupancyGrid::worldToGrid(float worldX, float worldY, int& gridX, int& gridY) const {
    int convertedX = std::floor((worldX - originX_) / resolution_);
    int convertedY = std::floor((worldY - originY_) / resolution_);

    if (!isInBounds(convertedX, convertedY)) {
        return false;
    }

    gridX = convertedX;
    gridY = convertedY;
    return true;
}

bool OccupancyGrid::gridToWorld(int gridX, int gridY, float& worldX, float& worldY) const {
    if (!isInBounds(gridX, gridY)) {
        return false;
    } 

    worldX = originX_ + (gridX + 0.5) * resolution_;
    worldY = originY_ + (gridY + 0.5) * resolution_;

    return true;
}

std::string OccupancyGrid::toString(int centerX, int centerY, int radius) const {
    
    std::string output; 

    for (int y = (centerY + radius); y >= (centerY - radius); y--) {
        for (int x = (centerX - radius); x <= (centerX + radius); x++) {
            if (!isInBounds(x, y)) {
                output += " ";
                continue;
            }

            CellState cell = getCell(x, y);

            if (cell == CellState::Unknown) {
                output += '?';
            } 
            
            else if (cell == CellState::Free) {
                output += '.';
            }

            else if (cell == CellState::Occupied) {
                output += '#';
            }
        }
        output += '\n';
    }

    return output;
}