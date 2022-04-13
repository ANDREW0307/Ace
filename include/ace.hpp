#include "main.h"


#define RAMPING_HEIGHT 200 
#define CLEAR_RAMP_HEIGHT 1000

void begin_task(std::string name, void (*func)(void*));
void pause_task(std::string name);
void kill_task(std::string name);


void set_drive_hold();
void set_drive_coast();
void reset_drive();
void set_drive_voltage(double voltageLeft, double voltageRight);


void front_let_go();
void front_clamp_on();
void back_let_go();
void back_clamp_on();