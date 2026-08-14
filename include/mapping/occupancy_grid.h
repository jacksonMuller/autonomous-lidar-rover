#pragma once
#include <vector>
#include <string>


enum class CellState {
    Unknown,
    Free,
    Occupied
}; 


class OccupancyGrid {

public:
    OccupancyGrid(int width, int height, float resolution);
    int getWidth() const; 
    int getHeight() const;
    float getResolution() const;

    CellState getCell(int x, int y) const;

    void setCell(int x, int y, CellState state);

    bool isInBounds(int x, int y) const;

    bool worldToGrid(float worldX, float worldY, int& gridX, int& gridY) const;

    bool gridToWorld(int gridX, int gridY, float& worldX, float& worldY) const;

    std::string toString(int centerX, int centerY, int radius) const;

private: 
    int width_; 
    int height_;
    float resolution_;
    float originX_;
    float originY_;

    std::vector<CellState> cells_; 

}; 