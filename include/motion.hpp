#include "main.h"

#define TYPE_MOTION_PROFILE 0
#define TYPE_PID 1 

void drive_straight_line_PID();
void turn_to_angle();
void move_to_point(std::vector<double> points, std::vector<double> margins, int mode, int time, std::vector<double> gains);

void myPIDstraight(int ticks, int tolerance, double kP, double kI, double kD, int degrees, double time);
void myPIDturn(int degrees, double kP, double kD, double time);