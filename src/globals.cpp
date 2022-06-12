#include "main.h"

pros::Motor frontLeft(14, pros::E_MOTOR_GEARSET_18, false);
pros::Motor midLeft(13, pros::E_MOTOR_GEARSET_18, true);
pros::Motor backLeft(15, pros::E_MOTOR_GEARSET_18, true);

pros::Motor frontRight(17, pros::E_MOTOR_GEARSET_18, true);
pros::Motor midRight(19, pros::E_MOTOR_GEARSET_18, false);
pros::Motor backRight(18, pros::E_MOTOR_GEARSET_18, false);

pros::Motor arm_motor(16, pros::E_MOTOR_GEARSET_18, true);

pros::Motor conveyor(20, pros::E_MOTOR_GEARSET_06, true);

pros::IMU inertial(12);

pros::ADIDigitalOut tilter_pistons('G'); 
pros::ADIDigitalOut back_claw_piston('F'); 
pros::ADIDigitalOut front_claw_piston1('E');
pros::ADIDigitalOut front_claw_piston2('H');


// quad encoders
pros::ADIEncoder leftEncoder ('A', 'B', true);
pros::ADIEncoder midEncoder ({11, 'A', 'B'}, false);
pros::ADIEncoder rightEncoder ('C', 'D', false);

pros::Controller master(pros::E_CONTROLLER_MASTER);