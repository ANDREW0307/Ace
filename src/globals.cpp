#include "main.h"

pros::Motor frontLeft(14, pros::E_MOTOR_GEARSET_18, false);
pros::Motor midLeft(13, pros::E_MOTOR_GEARSET_18, true);
pros::Motor backLeft(15, pros::E_MOTOR_GEARSET_18, true);

pros::Motor frontRight(17, pros::E_MOTOR_GEARSET_18, true);
pros::Motor midRight(19, pros::E_MOTOR_GEARSET_18, false);
pros::Motor backRight(18, pros::E_MOTOR_GEARSET_18, false);

pros::Motor arm_motor(16, pros::E_MOTOR_GEARSET_18, true);

pros::Motor conveyor(20, pros::E_MOTOR_GEARSET_06, false);

pros::Motor inertial(7);

pros::ADIDigitalOut tilter_pistons('G'); 
pros::ADIDigitalOut back_claw_piston('F'); 
pros::ADIDigitalOut front_claw_piston('E');

// quad encoders
pros::ADIEncoder leftEncoder ('E', 'F', false);
pros::ADIEncoder midEncoder ('G', 'H', false);
pros::ADIEncoder rightEncoder ({6, 'C', 'D'}, true);

pros::Controller master(pros::E_CONTROLLER_MASTER);