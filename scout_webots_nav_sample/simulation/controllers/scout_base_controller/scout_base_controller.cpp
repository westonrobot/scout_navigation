// File:          scout_basic_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>



// All the webots classes are defined in the "webots" namespace
using namespace webots;

#define TIME_STEP 64

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  
  // Lidar *lidar = robot->getLidar("rslidar_16");
  // lidar->enablePointCloud();
  // lidar->enable(20);
  
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  Motor *wheels[4];
  char wheels_names[4][20] = {"front_right_wheel", "front_left_wheel", "rear_left_wheel", "rear_right_wheel"};
  for (int i = 0; i < 4; i++) {
    wheels[i] = robot->getMotor(wheels_names[i]);
    wheels[i]->setPosition(INFINITY);
    wheels[i]->setVelocity(0.0);
  }
  
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
    double leftSpeed = 1.8;
    double rightSpeed = 1.8;
    wheels[0]->setVelocity(rightSpeed);
    wheels[1]->setVelocity(leftSpeed);
    wheels[2]->setVelocity(leftSpeed);
    wheels[3]->setVelocity(rightSpeed);
    // wheels[1]->setVelocity(leftSpeed);
    // wheels[2]->setVelocity(leftSpeed);
    // wheels[3]->setVelocity(rightSpeed);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
