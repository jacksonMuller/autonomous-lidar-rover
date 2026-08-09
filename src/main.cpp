#include <csignal>
#include <string>
#include "../include/utils/i2c.h"
#include "../include/motor/motor.h"
#include "../include/lidar/lidar.h"
#include "../include/nav/navigator.h"



const std::string I2C_PATH = ""; 
const uint8_t ADDRESS = 5; 
const std::string PORT_PATH = ""; 


bool running = true; 

void signalHandler(int signal) {
    running = false; 
}

int main() {
    std::signal(SIGINT, signalHandler);

    // Create objects
    I2C device{I2C_PATH, ADDRESS}; 
    MotorController controller{device}; 
    LidarParser parser{PORT_PATH};
    Navigator nav{controller};


    // Daemon loop
    while (running) {
        LidarScan scan = parser.getLidarData();
        nav.update(scan);
    } 

    controller.stop(); 
}