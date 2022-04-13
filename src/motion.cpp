#include "main.h"

int velCap;
void myPIDstraight(int ticks, int tolerance, double kP, double kI, double kD, int degrees, double time)
{

  int startTime = pros::millis();
  int currentTime = pros::millis();

  int deltaTime = currentTime - startTime;

    int signLeft;
    int signRight;

    

           leftEncoder.reset();
           rightEncoder.reset();
           midEncoder.reset();

		   	 double errorLeft = ticks - leftEncoder.get_value();
 double errorRight = ticks - rightEncoder.get_value();
	double prevErrorLeft = 0;
	double prevErrorRight = 0;

 while ( fabs(errorLeft) > tolerance && fabs(errorRight) > tolerance && deltaTime < time)
 {

    currentTime = pros::millis();
    deltaTime = currentTime - startTime;

        signLeft = (errorLeft > 0) - (errorLeft < 0); // + or -
        signRight = (errorRight > 0) - (errorRight < 0);

  errorLeft = ticks - leftEncoder.get_value();
  errorRight = ticks - rightEncoder.get_value();

 double integralLeft = integralLeft + errorLeft;
 double integralRight = integralRight + errorRight;




 if (errorLeft == 0 or leftEncoder.get_value() > ticks) {
 integralLeft = 0;
 }
 if (errorLeft > 20){
 integralLeft = 0;
 }

 if (errorRight == 0 or rightEncoder.get_value() > ticks) {
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

// if (abs(inertial.get_rotation()) > 4) {
//     powerLeft -= ;
//     powerRight += inertial.get_rotation() * 2;
// }

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

 frontLeft = powerLeft - (degrees_error * 3.5);
 midLeft = powerLeft - (degrees_error * 3.5) ;
 backLeft = powerLeft - (degrees_error * 3.5);

 frontRight = powerRight + (degrees_error * 3.5);
 midRight = powerRight + (degrees_error * 3.5);
 backRight = powerRight + (degrees_error * 3.5);
//  if(frontLeft.get_current_draw() < 100 && frontRight.get_current_draw() < 100) {
// 	 break;
//  }
 pros::delay(20);
 }
 set_drive_voltage(0,0);
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

void move_to_point(std::vector<double> points, std::vector<double> margins, int mode, int time, std::vector<double> gains) {

    double targetX = points[0] * INCHES_TO_TICKS_CONSTANT_LRS; 
    double targetY = points[1] * INCHES_TO_TICKS_CONSTANT_LRS;   


    double currentX = gPosition.x * INCHES_TO_TICKS_CONSTANT_LRS;
    double currentY = gPosition.y * INCHES_TO_TICKS_CONSTANT_LRS;

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
    
    if(fabs(error_theta > 20)) {
        myPIDturn(error_theta, 1, .1, 1000);
    }

    switch (mode)
    {
    case 0: // motion profile
        /* code */
        break;
    case 1: // PID
        while (fabs(targetY - gPosition.y) > margins[1] && deltaTime < time)
        {
            currentX = gPosition.x * INCHES_TO_TICKS_CONSTANT_LRS;
            currentY = gPosition.y * INCHES_TO_TICKS_CONSTANT_LRS;


            // h_mag = sqrt(pow(targetX - gPosition.x, 2) + pow(targetY - gPosition.y, 2)); // distance formula
            currentTime = pros::millis();
            deltaTime = currentTime - startTime;

            double kP_x = gains[0];
            double kI_x = gains[1];
            double kD_x = gains[2];
            
            double kP_y = gains[3];
            double kI_y = gains[4];
            double kD_y = gains[5];


            double error_x;
            double error_y;

            double power_x;
            double power_y;

            double prevError_x;
            double prevError_y;

            // X AXIS PID
            if (fabs(targetX - currentX) > margins[0])
            {
                error_x = targetX - gPosition.x;
                double deriv_x = error_x - prevError_x;

                power_x = (error_x * kP_x) + (deriv_x * kD_x);
                prevError_x = error_x; 
            }
            
            // Y AXIS PID
            if (fabs(targetY - currentY) > margins[1])
            {

                error_y = targetY - currentY;
                double deriv_y = error_y - prevError_y;
                
                power_y = (error_y * kP_y) + (deriv_y * kD_y);
                prevError_y = error_y;
                
            }
            
            set_drive_voltage(power_y + power_x, power_y - power_x);

            pros::delay(10);
        }
        set_drive_voltage(0,0);
        break;
    default:
        break;
    }
    
}
