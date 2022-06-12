#include "main.h"


using namespace okapi;

std::shared_ptr<ChassisController> myChassis =
  ChassisControllerBuilder()
    .withMotors({14, 13, 15}, {17, 19, 18})
    // Green gearset, 4 in wheel diam, 11.5 in wheel track
    .withDimensions({AbstractMotor::gearset::green, 5.0/7.0}, {{4.125_in, 13.4375_in}, imev5GreenTPR})
    .build();

std::shared_ptr<AsyncMotionProfileController> profileController =
  AsyncMotionProfileControllerBuilder()
    .withLimits({
      0.75, // Maximum linear velocity of the Chassis in m/s
      1.25, // Maximum linear acceleration of the Chassis in m/s/s
      7.0 // Maximum linear jerk of the Chassis in m/s/s/s
    })
    .withOutput(myChassis)
    .buildMotionProfileController();


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

std::atomic<bool> front_claw_on;
std::atomic<bool> back_claw_on;
std::atomic<bool> tilter_on;
std::atomic<bool> back_mech_out;

void back_mogo_control(void *ptr) {
	
	while(true) {
		// back mech combined control
		bool wasPressedBackMech = master.get_digital_new_press(DIGITAL_L2);

		if (wasPressedBackMech && back_mech_out) {
			tilter_pistons.set_value(false);
			back_claw_piston.set_value(false);
			back_mech_out = false;
		} else if (wasPressedBackMech && !back_mech_out) {
			back_claw_piston.set_value(true);
			pros::delay(170);
			tilter_pistons.set_value(true);
			back_mech_out = true;			
		}
		pros::delay(20);
	}

}

void initialize() {
	set_drive_coast();
	pros::lcd::initialize();
	
	leftEncoder.reset();
	midEncoder.reset();
	rightEncoder.reset();
	begin_task("backMogoTask", back_mogo_control);


}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	set_drive_coast();
	arm_motor.set_brake_mode(MOTOR_BRAKE_HOLD);

	inertial.reset();
	while (inertial.is_calibrating()) 
	{
		pros::delay(10);
	}
	

	leftEncoder.reset();
	midEncoder.reset();
	rightEncoder.reset();

	back_let_go();
	front_let_go();

	begin_task("tracking", positionTracking);
	arm_motor.tare_position();
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */




void autonomous() {



	// TAX FRAUD RIGHT



	arm_motor = 127;
	front_let_go();
	back_let_go();
	pros::delay(300);
	arm_motor.move_absolute(-100, 600);


	move_to_point_PID({0,51, 0}, {2, .25, 5}, {{0.01, 0}, {0.13, 0.05}, {0.1, 0}}, 10000);
	front_clamp_on();
	pros::delay(200);
	arm_motor.move_absolute(ABOVE_GROUND, 600);
	move_to_point_PID({0, 5, 0}, {2, .25, 5}, {{0.01, 0}, {0.16, 0.15}, {0.1, 0}}, 1000);
	myPIDturn(-80, 4, .1, 1000);
	arm_motor.move_absolute(200, 600);
	myPIDstraight(600, 50, .2, 0, .2, inertial.get_rotation(), 1000);
	front_let_go();
	arm_motor.move_absolute(-100, 600);
	pros::delay(300);
	myPIDstraight(-200, 30, .2, 0, .1, inertial.get_rotation(), 1000);
	myPIDturn(-5, 4, .05, 1000);
	// move_to_point_PID({24, 44, -45}, {1, 0.5, 1}, {{.01, 0.1}, {1.4, 0.09}, {.0001, 0}}, 4000);
		// positive is to the left


	QAngle angle = fabs(inertial.get_rotation()) * degree;
	QLength posX = totalPositionX * inch;
	QLength posY = totalPositionY * inch;

	// okapi conversion task so that these can be used whenever i want motion profiling
	// add another mode to move to point

	profileController->generatePath(
		// {{posX, posY, 0_deg}, {24_in, 24_in, 0_deg}}, "A");
		{{posY, posX, angle}, {45_in, 38_in, 50_deg}}, "A");

	profileController->setTarget("A");
	profileController->waitUntilSettled();

	front_clamp_on();
	arm_motor.move_absolute(ABOVE_GROUND, 600);


	 angle = fabs(inertial.get_rotation()) * degree;
	 posX = totalPositionX * inch;
	 posY = totalPositionY * inch;

	profileController->generatePath(
		// {{40_in, 20_in, 45_deg}, {18_in, -20_in, 90_deg}}, "B");
		{{posY, posX, angle}, {25_in, -20_in, 90_deg}}, "B");


	profileController->setTarget("B", true, true);
	profileController->waitUntilSettled();

	set_drive_voltage(-80, -50);
	pros::delay(600);
	set_drive_voltage(0,0);
	back_clamp_on();
	arm_motor.move_absolute(ABOVE_GROUND + 300, 600);
	conveyor.move_velocity(600);

	set_drive_voltage(70, 70);
	pros::delay(200);
	myPIDstraight(1400, 20, .2, 0, .2, 0, 3000);
	myPIDstraight(-1400, 20, .2, 0, .2, 0, 3000);

	back_let_go();





	
}


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {


		arm_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
	set_drive_coast();

	int tilter_control_value = 0;
	int back_claw_control_value = 0;

	front_clamp_on();

	back_claw_piston.set_value(false);
	tilter_pistons.set_value(false);

	front_claw_on = true;
	back_claw_on = false;
	tilter_on = true;
	back_mech_out = true;

	int brakeType_toggle = 0;

	while (true) {
		// pros::lcd::print(4, "GOD BLESS ARSENAL");


		bool wasPressedTilter = master.get_digital_new_press(DIGITAL_DOWN);

		if (wasPressedTilter && tilter_on) {
			tilter_pistons.set_value(false);
			tilter_on = false;
		} else if (wasPressedTilter && !tilter_on) {
			tilter_pistons.set_value(true);
			tilter_on = true;
		}


		bool wasPressedBack = master.get_digital_new_press(DIGITAL_LEFT);
		if (wasPressedBack && back_claw_on) {
			back_claw_piston.set_value(false);
			back_claw_on = false;
		} else if (wasPressedBack && !back_claw_on) {
			back_claw_piston.set_value(true);
			back_claw_on = true;
		}



		bool wasPressedFront = master.get_digital_new_press(DIGITAL_L1);

		if (wasPressedFront && front_claw_on) {
			front_let_go();
			front_claw_on = false;
		} else if (wasPressedFront && !front_claw_on) {
			front_clamp_on();
			front_claw_on = true;
		}



		double fowardsInput = master.get_analog(ANALOG_LEFT_Y);
		double turnInput = master.get_analog(ANALOG_RIGHT_X); 

		double left = fowardsInput + turnInput;
		double right = fowardsInput - turnInput;

		frontLeft = left;
		midLeft = left;
		backLeft = left;

		frontRight = right;
		midRight = right;
		backRight = right;

		
				// conveyor control - bound to the right arrow and Y
		bool isPressedConveyorFWD = master.get_digital_new_press(DIGITAL_Y);
		bool isPressedConveyorREV = master.get_digital_new_press(DIGITAL_RIGHT);

		if(isPressedConveyorFWD && conveyor.get_target_velocity() == 0){
			conveyor.move_velocity(-600);
		} else if(isPressedConveyorFWD && conveyor.get_target_velocity() == -600){
			conveyor.move_velocity(0);
		} else if(isPressedConveyorREV && conveyor.get_target_velocity() == 0) {
			conveyor.move_velocity(600);
		} else if(isPressedConveyorREV && conveyor.get_target_velocity() == 600) {
			conveyor.move_velocity(0);
		} else if(isPressedConveyorFWD && conveyor.get_target_velocity() == 600) {
			conveyor.move_velocity(-600);
		} else if(isPressedConveyorREV && conveyor.get_target_velocity() == -600) {
			conveyor.move_velocity(600);
		}


		// arm control 
		if (master.get_digital(DIGITAL_R1))
		{
			arm_motor.move_velocity(200);
		} else if (master.get_digital(DIGITAL_R2)) {
			arm_motor.move_velocity(-200);
		} else {
			arm_motor.move_velocity(0);
		}



		bool wasPressedBrakeToggle = master.get_digital_new_press(DIGITAL_A);

		if (wasPressedBrakeToggle && brakeType_toggle == 0) {
			set_drive_hold();
			brakeType_toggle++;
		} else if (wasPressedBrakeToggle && brakeType_toggle == 1) {
			set_drive_coast();
			brakeType_toggle--;
		}


		
		pros::delay(20);
	}
}
