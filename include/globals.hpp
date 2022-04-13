#include "main.h"


// motors - 6 motor drive, 1 arm, 1 conveyor 
extern pros::Motor frontLeft;
extern pros::Motor midLeft;
extern pros::Motor backLeft;
extern pros::Motor frontRight;
extern pros::Motor midRight;
extern pros::Motor backRight;
extern pros::Motor arm_motor;
extern pros::Motor conveyor;

extern pros::IMU inertial; 

// piston solenoid control 
extern pros::ADIDigitalOut tilter_pistons;
extern pros::ADIDigitalOut back_claw_piston;
extern pros::ADIDigitalOut front_claw_piston1;
extern pros::ADIDigitalOut front_claw_piston2;


// quad encoders
extern pros::ADIEncoder leftEncoder;
extern pros::ADIEncoder midEncoder;
extern pros::ADIEncoder rightEncoder;

extern pros::Controller master;