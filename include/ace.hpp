#include "main.h"

void begin_task(std::string name, void (*func)(void*));
void pause_task(std::string name);
void kill_task(std::string name);


void set_drive_hold();
void set_drive_coast();
void reset_drive();
void set_drive_voltage(double voltageLeft, double voltageRight);