#include "main.h"

// TASK / THREADING CONTROL

std::vector<pros::Task> task_list;
std::vector<pros::Task>::iterator task_list_itr;

void begin_task(std::string name, void (*func)(void*)) {
      
    pros::Task myTask(func, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, name.c_str());
    // task_list.insert(task_list_itr, myTask);
}

// void pause_task(std::string name) {
//     for (std::size_t i = 0; i < task_list.size(); ++i) {
//         if(task_list[i].get_name() == name.c_str()) {
//             task_list[i].suspend();
//         }
//     }
// }

// void resume_task(std::string name) {
//     for (std::size_t i = 0; i < task_list.size(); ++i) {
//         if(task_list[i].get_name() == name.c_str()) {
//             task_list[i].resume();
//         }
//     }
// }

// void kill_task(std::string name) {
//     for (std::size_t i = 0; i < task_list.size(); ++i) {
//         if(task_list[i].get_name() == name.c_str()) {
//             task_list[i].remove();
//         }
//     }
// }


void set_drive_coast() {
	frontLeft.set_brake_mode(MOTOR_BRAKE_COAST);
    midLeft.set_brake_mode(MOTOR_BRAKE_COAST);
	backLeft.set_brake_mode(MOTOR_BRAKE_COAST);
	frontRight.set_brake_mode(MOTOR_BRAKE_COAST);
    midRight.set_brake_mode(MOTOR_BRAKE_COAST);
	backRight.set_brake_mode(MOTOR_BRAKE_COAST);
}

void set_drive_hold() {
	frontLeft.set_brake_mode(MOTOR_BRAKE_HOLD);
    midLeft.set_brake_mode(MOTOR_BRAKE_HOLD);
	backLeft.set_brake_mode(MOTOR_BRAKE_HOLD);
	frontRight.set_brake_mode(MOTOR_BRAKE_HOLD);
    midRight.set_brake_mode(MOTOR_BRAKE_HOLD);
	backRight.set_brake_mode(MOTOR_BRAKE_HOLD);
}


void set_drive_voltage(double voltageLeft, double voltageRight) {
    frontLeft = voltageLeft;
    midLeft = voltageLeft;
    backLeft = voltageLeft;
    frontRight = voltageRight;
    midRight = voltageRight;
    backRight = voltageRight;
}




void front_let_go() {
    front_claw_piston1.set_value(false);
    front_claw_piston2.set_value(false);
}
void front_clamp_on() {
    front_claw_piston1.set_value(true);
    front_claw_piston2.set_value(true);
}

void back_let_go();
void back_clamp_on();




