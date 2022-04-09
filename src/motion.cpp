#include "main.h"

int velCap;
void myPIDstraight(double targetLeft, double targetRight, int tolerance, double kP, double kI, double kD, int degrees, double time)
{

    double startTime = pros::millis();
    double currentTime = pros::millis();

    double deltaTime = currentTime - startTime;

    int signLeft;
    int signRight;

    targetLeft *= INCHES_TO_TICKS_CONSTANT_LRS;
    targetRight *= INCHES_TO_TICKS_CONSTANT_LRS;

	double errorLeft = targetLeft - leftEncoder.get_value();
    double errorRight = targetRight - rightEncoder.get_value();
	double prevErrorLeft = 0;
	double prevErrorRight = 0;

 while ( fabs(errorLeft) > tolerance && fabs(errorRight) > tolerance && deltaTime < time)
 {

    currentTime = pros::millis();
    deltaTime = currentTime - startTime;

        signLeft = (errorLeft > 0) - (errorLeft < 0); // + or -
        signRight = (errorRight > 0) - (errorRight < 0);

  errorLeft = targetLeft - leftEncoder.get_value();
  errorRight = targetRight - rightEncoder.get_value();

 double integralLeft = integralLeft + errorLeft;
 double integralRight = integralRight + errorRight;




 if (errorLeft == 0 or leftEncoder.get_value() > targetLeft) {
 integralLeft = 0;
 }
 if (errorLeft > 20){
 integralLeft = 0;
 }

 if (errorRight == 0 or rightEncoder.get_value() > targetRight) {
 integralRight = 0;
 }
 if (errorRight > 20){
 integralRight = 0;
 }

 double derivativeLeft = errorLeft - prevErrorLeft;
 double derivativeRight = errorRight - prevErrorRight;

 prevErrorLeft = errorLeft;
 prevErrorRight = errorRight;

 double powerLeft =  errorLeft*kP + integralLeft*kI + derivativeLeft*kD;
 double powerRight =  errorRight*kP + integralRight*kI + derivativeRight*kD;


        int acc = 8;
        velCap = velCap + acc;  //slew rate
        if(velCap > 95){
          velCap = 95; //velCap cannot exceed 120
        }
        if(abs(powerLeft) > velCap){ //limit the voltage
          powerLeft = velCap * signLeft;
        }
        if(abs(powerRight) > velCap){ //ditto
          powerRight = velCap * signRight;
        }

    double degrees_error = inertial.get_rotation() - degrees;

    set_drive_voltage(powerLeft - (degrees_error * 4), powerRight + (degrees_error * 4));


 pros::delay(20);
 }
 reset_drive();
}

void myPIDturn(int degrees, double kP, double kD, double time)
{

    set_drive_hold();


    int startTime = pros::millis();
    int currentTime = pros::millis();

    int deltaTime = currentTime - startTime;

	 double error = degrees - inertial.get_rotation();

	double prevError = 0;
 while ( fabs(error) > 1 && deltaTime < time)
 {

       currentTime = pros::millis();
    deltaTime = currentTime - startTime;

  error = degrees - inertial.get_rotation();
 double derivative = error - prevError;
 prevError = error;

 double power =  error*kP + derivative*kD;

 frontLeft = power;
 backLeft = power;
 
 frontRight = -power;
 backRight = -power;

 pros::delay(20);

 }

 
 set_drive_coast();
 set_drive_voltage(0,0);

}

void move_to_point(std::vector<double> points, std::vector<double> margins, int mode, int time) {

    double targetX = INCHES_TO_TICKS_CONSTANT_LRS * points[0]; 
    double targetY = INCHES_TO_TICKS_CONSTANT_LRS * points[1];   

    // calculating the triangle
    double h_mag = sqrt(pow(targetX - gPosition.x, 2) + pow(targetY - gPosition.y, 2)); // distance formula
    double theta_triangle = atan2(fabs(targetY - gPosition.y), fabs(targetX - gPosition.x));

    int startTime = pros::millis();
    int currentTime = pros::millis();
    int deltaTime = currentTime - startTime;
    

    double error_theta;

    int quadrant;

    if (targetX > gPosition.x) {
        if (targetY > gPosition.y)
        {
            quadrant = 1;
        } else {
            quadrant = 4;
        }
    } else {
        if (targetY > gPosition.y)
        {
            quadrant = 2;
        } else {
            quadrant = 3;
        }
    }

    switch (quadrant)
    {
    case 1:
        error_theta = 90 - theta_triangle;
        break;
    case 2:
        error_theta = theta_triangle - 90;
        break;
    case 3:
        error_theta = -theta_triangle - 90;
        break;
    case 4:
        error_theta = 90 + theta_triangle;
        break;

    default:
        break;
    }

    myPIDturn(error_theta, 1, .1, 1000);

    switch (mode)
    {
    case 0: // motion profile
        /* code */
        break;
    case 1: // PID
        while (fabs(targetX - gPosition.x) > margins[0] && fabs(targetY - gPosition.y) > margins[1] && deltaTime < time)
        {
            // h_mag = sqrt(pow(targetX - gPosition.x, 2) + pow(targetY - gPosition.y, 2)); // distance formula
            currentTime = pros::millis();
            deltaTime = currentTime - startTime;

            double kP_x = .2;
            double kP_y = .5;

            double error_x;
            double power_x;
            double error_y;
            double power_y;

            // X AXIS PID
            if (fabs(targetX - gPosition.x) > margins[0])
            {
                double error_x = targetX - gPosition.x;
                double power_x = error_x * kP_x;
            }
            
            // Y AXIS PID
            if (fabs(targetY - gPosition.y) > margins[0])
            {
                double error_y = targetY - gPosition.y;
                double power_y = error_y * kP_y;
            }
            
            set_drive_voltage(power_y + power_x, power_y - power_x);
            pros::delay(10);
        }
        break;
    default:
        break;
    }
    
}
