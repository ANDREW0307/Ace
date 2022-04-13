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
	
	set_drive_coast();
	pros::lcd::initialize();
	
	begin_task("backMogoTask", back_mogo_control);
	begin_task("tracking", positionTracking);

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

	arm_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
	back_claw_piston.set_value(false);
	tilter_pistons.set_value(true);
	// move_to_point({0, 20, 0}, {0, 1, 0}, 1, 13000, {0, 0, 0, .2, 0, 0});
	// right side tax fraud

	// front_claw_piston.set_value(false);
	// tilter_pistons.set_value(true);
	// back_claw_piston.set_value(false);
	// arm_motor.move_relative(-100, 100);
	// arm_motor.tare_position();
	// myPIDstraight(1940, 1, .29, 0, 1.45, 0, 1400);
	// front_claw_piston.set_value(true);
	// arm_motor.move_absolute(600, 200);
	// pros::delay(200);
	arm_motor.move_absolute(-100, 200);
	// front_claw_piston.set_value(false);
	set_drive_voltage(127,127);
	pros::delay(890);
	set_drive_voltage(0,0);
	pros::delay(100);
	// front_claw_piston.set_value(true);
	arm_motor.move_absolute(2000, 200);
	myPIDstraight(-1000, 10, .4, 0, .1, 10, 2000);
	


	

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

	front_let_go();

	back_claw_piston.set_value(false);
	tilter_pistons.set_value(false);

	front_claw_on = false;
	back_claw_on = false;
	tilter_on = true;
	back_mech_out = true;

	int brakeType_toggle = 0;


	while (true) {
		// pros::lcd::print(4, "GOD BLESS ARSENAL");
		// pros::lcd::set_text(5, "left encoder" + std::to_string(leftEncoder.get_value()));
		// pros::lcd::set_text(6, "right encoder" + std::to_string(rightEncoder.get_value()));
		// pros::lcd::set_text(7, "mid encoder" + std::to_string(midEncoder.get_value()));

		set_drive_coast();


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



		bool wasPressedBrakeToggle = master.get_digital_new_press(DIGITAL_B);

		if (wasPressedBrakeToggle && brakeType_toggle % 2 == 0) {
			set_drive_hold();
			brakeType_toggle++;
		} else if (wasPressedBrakeToggle && brakeType_toggle % 2 == 1) {
			set_drive_coast();
			brakeType_toggle++;
		}


		
		pros::delay(20);
	}
}
