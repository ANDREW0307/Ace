#include "main.h"





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
	pros::lcd::initialize();
	
	begin_task("backMogoTask", back_mogo_control);
	begin_task("tracking", positionTracking);

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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
void autonomous() {}


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

	front_claw_piston.set_value(false);
	back_claw_piston.set_value(false);
	tilter_pistons.set_value(false);

	front_claw_on = false;
	back_claw_on = false;
	tilter_on = true;
	back_mech_out = true;

	while (true) {
		pros::lcd::print(4, "GOD BLESS ARSENAL");

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
			front_claw_piston.set_value(false);
			front_claw_on = false;
		} else if (wasPressedFront && !front_claw_on) {
			front_claw_piston.set_value(true);
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
		bool isPressedConveyorFWD = master.get_digital_new_press(DIGITAL_RIGHT);
		bool isPressedConveyorREV = master.get_digital_new_press(DIGITAL_Y);

		if(isPressedConveyorFWD && conveyor.get_target_velocity() == 0){
			conveyor.move_velocity(600);
		} else if(isPressedConveyorFWD && conveyor.get_target_velocity() == 600){
			conveyor.move_velocity(0);
		} else if(isPressedConveyorREV && conveyor.get_target_velocity() == 0) {
			conveyor.move_velocity(-600);
		} else if(isPressedConveyorREV && conveyor.get_target_velocity() == -600) {
			conveyor.move_velocity(0);
		} else if(isPressedConveyorFWD && conveyor.get_target_velocity() == -600) {
			conveyor.move_velocity(600);
		} else if(isPressedConveyorREV && conveyor.get_target_velocity() == 600) {
			conveyor.move_velocity(-600);
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

		
		pros::delay(20);
	}
}
