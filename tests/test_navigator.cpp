#include <catch2/catch_test_macros.hpp>

#include "nav/navigator.h"
#include "motor/i_motor_controller.h"
#include "sim/scan_injector.h"
#include "utils/types.h"

#include <iostream>

class FakeMotorController : public IMotorController {
public:
    enum class Command {
        None,
        Forward,
        TurnLeft,
        TurnRight,
        Stop
    };

    void forward(float speed) override {
        lastCommand = Command::Forward;
        lastSpeed = speed;
    }

    void turnLeft(float speed) override {
        lastCommand = Command::TurnLeft;
        lastSpeed = speed;
    }

    void turnRight(float speed) override {
        lastCommand = Command::TurnRight;
        lastSpeed = speed;
    }

    void stop() override {
        lastCommand = Command::Stop;
        lastSpeed = 0.0f;
    }

    Command lastCommand = Command::None;
    float lastSpeed = 0.0f;
};


// Test #6
TEST_CASE("Navigator stops on invalid scan") {
    LidarScan scan{}; 
    scan.validScan = false;

    NavigationResult result = Navigator::evaluateScan(scan);

    REQUIRE(result.decision == NavDecision::Stop);
    REQUIRE(result.status == NavStatus::SensorFault);
}

// Test #7 
TEST_CASE("Navigator moves forward when path is clear") {
    LidarScan scan = ScanInjector::allClear();

    NavigationResult result = Navigator::evaluateScan(scan); 

    REQUIRE(result.decision == NavDecision::Forward);
    REQUIRE(result.status == NavStatus::Normal);

}

// Test #8
TEST_CASE("Navigator turns left when left side has more clearance") {
    LidarScan scan{}; 

    // Populate scan distances to unsafe distance (0.3f)
    scan.distances.fill(0.3f);
    // Populate intensity readings to 150 (good reads)
    scan.intensity.fill(150); 
    // Set validScan to true
    scan.validScan = true; 

    // Set left distances to 10.0f (45 to 135 deg) (safe distance)
    for (int i = 45; i <= 135; i++) {
        scan.distances[i] = 10.0f; 
    }

    NavigationResult result = Navigator::evaluateScan(scan); 

    REQUIRE(result.decision == NavDecision::TurnLeft);
    REQUIRE(result.status == NavStatus::Normal); 
}

// Test #9
TEST_CASE("Navigator turns right when right side has more clearance") {
    LidarScan scan{}; 

    // Populate scan distances to unsafe distance (0.3f)
    scan.distances.fill(0.3f);
    // Populate intensity readings to 150 (good reads)
    scan.intensity.fill(150); 
    // Set validScan to true
    scan.validScan = true; 

    // Set right distances to 10.0f (225 to 315 deg) (safe distance)
    for (int i = 225; i <= 315; i++) {
        scan.distances[i] = 10.0f;
    }

    NavigationResult result = Navigator::evaluateScan(scan);

    REQUIRE(result.decision == NavDecision::TurnRight);
    REQUIRE(result.status == NavStatus::Normal);
    
}

// Test #10 
TEST_CASE("Navigator stops when all sides are blocked") {
    LidarScan scan = ScanInjector::surrounded(); 

    NavigationResult result = Navigator::evaluateScan(scan);


    std::cerr
        << "Decision: " << static_cast<int>(result.decision)
        << " Status: " << static_cast<int>(result.status)
        << '\n';

    
    REQUIRE(result.decision == NavDecision::Stop); 
    REQUIRE(result.status == NavStatus::Normal);
}

// Test #11
TEST_CASE("Navigator stops when forward scan contains no valid readings") {
    LidarScan scan = ScanInjector::allClear();
    scan.validScan = true;

    // Now set forward distances to 0.3f (0 to 45 deg)
    for (int i = 0; i <= 45; i++) {
        scan.distances[i] = 0.0f;
        scan.intensity[i] = 0;  
    }
    // And 315 to 359 deg
    for (int i = 315; i <= 359; i++) {
        scan.distances[i] = 0.0f;
 
    }

    NavigationResult result = Navigator::evaluateScan(scan); 

    REQUIRE(result.decision == NavDecision::Stop);
    REQUIRE(result.status == NavStatus::SensorFault); 
}

// Test #12
TEST_CASE("Navigator starts in Idle state") {
    FakeMotorController motors; 
    Navigator navigator(motors); 

    // Ensure inital state is set to Idle
    REQUIRE(navigator.getState() == RoverState::Idle);

}

// Test #13
TEST_CASE("Navigator enters Navigating state on clear path") {
    FakeMotorController motors;
    Navigator navigator(motors); 

    // Inject scan with clear path
    LidarScan scan = ScanInjector::allClear();

    // Update navigator with clear scan
    navigator.update(scan);
    
    // Ensure navigator chooses correct state and command
    REQUIRE(navigator.getState() == RoverState::Navigating); 
    REQUIRE(motors.lastCommand == FakeMotorController::Command::Forward);

}

// Test #14
TEST_CASE("Navigator enters AvoidingObstacle state when turning left") {
    FakeMotorController motors; 
    Navigator navigator(motors); 

    LidarScan scan = ScanInjector::surrounded(); 

    // Make left side safe to turn into
    for (int i = 45; i <= 135; i++) {
        scan.distances[i] = 10.0f; 
    }

    navigator.update(scan); 

    // Ensure navigator chooses correct state and command
    REQUIRE(navigator.getState() == RoverState::AvoidingObstacle);
    REQUIRE(motors.lastCommand == FakeMotorController::Command::TurnLeft);

}

// Test #15
TEST_CASE("Navigator enters AvoidingObstacle state when turning right") {
    FakeMotorController motors; 
    Navigator navigator(motors); 
    
    LidarScan scan = ScanInjector::surrounded(); 

    // Make right side safe to turn into
    for (int i = 225; i <= 315; i++) {
        scan.distances[i] = 10.0f; 
    }

    navigator.update(scan); 

    // Ensure navigator chooses correct state and command
    REQUIRE(navigator.getState() == RoverState::AvoidingObstacle);
    REQUIRE(motors.lastCommand == FakeMotorController::Command::TurnRight);
}

// Test #16
TEST_CASE("Navigator enters Stopped state when surrounded") {
    FakeMotorController motors; 
    Navigator navigator(motors); 

    LidarScan scan = ScanInjector::surrounded(); 

    navigator.update(scan); 

    // Ensure navigator chooses correct state and command 
    REQUIRE(navigator.getState() == RoverState::Stopped);
    REQUIRE(motors.lastCommand == FakeMotorController::Command::Stop); 
}

// Test #17
TEST_CASE("Navigator transitions from Navigating to AvoidingObstacle") {
    FakeMotorController motors; 
    Navigator navigator(motors); 

    LidarScan scan = ScanInjector::allClear(); 

    navigator.update(scan);

    // Ensure navigator chooses correct state and command
    REQUIRE(navigator.getState() == RoverState::Navigating); 
    REQUIRE(motors.lastCommand == FakeMotorController::Command::Forward);

    // Create new scan that requires rover to turn left
    LidarScan secondScan = ScanInjector::surrounded(); 

    // Make left side safe to turn into
    for (int i = 45; i <= 135; i++) {
        secondScan.distances[i] = 10.0f; 
    }

    navigator.update(secondScan); 

    // Ensure navigator chooses correct state and command
    REQUIRE(navigator.getState() == RoverState::AvoidingObstacle);
    REQUIRE(motors.lastCommand == FakeMotorController::Command::TurnLeft); 
}

// Test #18
TEST_CASE("Navigator transitions from AvoidingObstacle to Navigating") {
    FakeMotorController motors; 
    Navigator navigator(motors); 

    LidarScan scan = ScanInjector::surrounded();

    // Make right side safe to turn into
    for (int i = 225; i <= 315; i++) {
        scan.distances[i] = 10.0f; 
    }

    navigator.update(scan); 

    // Ensure navigator chooses correct state and command
    REQUIRE(navigator.getState() == RoverState::AvoidingObstacle);
    REQUIRE(motors.lastCommand == FakeMotorController::Command::TurnRight); 

    // Create new scan for rover to go straight
    LidarScan secondScan = ScanInjector::allClear();

    navigator.update(secondScan); 

    // Ensure navigator chooses correct state and command
    REQUIRE(navigator.getState() == RoverState::Navigating);
    REQUIRE(motors.lastCommand == FakeMotorController::Command::Forward);
    

}

// Test #19
TEST_CASE("Navigator enters SensorFault state on invalid scan") {
    FakeMotorController motors; 
    Navigator navigator(motors); 

    LidarScan scan = ScanInjector::allClear(); 
    // Make the scan invalid
    scan.validScan = false; 

    navigator.update(scan);

    // Ensure motor lastCommand is stop and rover state is SensorFault
    REQUIRE(motors.lastCommand == FakeMotorController::Command::Stop);
    REQUIRE(navigator.getState() == RoverState::SensorFault);

}


// TEST #20
TEST_CASE("Navigator recovers from SensorFault when scan becomes valid") {
    FakeMotorController motors; 
    Navigator navigator(motors); 

    LidarScan badScan = ScanInjector::allClear(); 
    badScan.validScan = false; 

    navigator.update(badScan); 

    // Ensure motor lastCommand is stop and rover state is SensorFault
    REQUIRE(motors.lastCommand == FakeMotorController::Command::Stop); 
    REQUIRE(navigator.getState() == RoverState::SensorFault); 

    LidarScan goodScan = ScanInjector::allClear();

    navigator.update(goodScan); 

    // Ensure motor lastCommand is Forward and rover state is Navigating
    REQUIRE(motors.lastCommand == FakeMotorController::Command::Forward); 
    REQUIRE(navigator.getState() == RoverState::Navigating); 
}

