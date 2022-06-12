#include "main.h"

int velCap;
void myPIDstraight(int ticks, int tolerance, double kP, double kI, double kD, int degrees, double time)
{

    double startTime = pros::millis();
    double currentTime = pros::millis();
    double deltaTime = currentTime - startTime;

    int signLeft, signRight;

    double target_left = ticks + leftEncoder.get_value();
    double target_right = ticks + rightEncoder.get_value();


	double errorLeft = target_left - leftEncoder.get_value();
    double errorRight = target_right - rightEncoder.get_value();
	
    double prevErrorLeft, prevErrorRight = 0;

    while ( fabs(errorLeft) > tolerance && fabs(errorRight) > tolerance && deltaTime < time) {

    currentTime = pros::millis();
    deltaTime = currentTime - startTime;

    signLeft = (errorLeft > 0) - (errorLeft < 0); // + or -
    signRight = (errorRight > 0) - (errorRight < 0);

    errorLeft = target_left - leftEncoder.get_value();
    errorRight = target_right - rightEncoder.get_value();

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

    int acc = 8;
    velCap = velCap + acc;  //slew rate
    if(velCap > 95) {
        velCap = 95; //velCap cannot exceed 120
    }

    if(abs(powerLeft) > velCap) { //limit the voltage
        powerLeft = velCap * signLeft;
    }

    if(abs(powerRight) > velCap) { 
        powerRight = velCap * signRight;
    }

    double degrees_error = inertial.get_rotation() - degrees;

    set_drive_voltage(powerLeft - (degrees_error * 3.5), powerRight + (degrees_error * 3.5));

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

    double targetX = points[0] * INCHES_TO_TICKS; 
    double targetY = points[1] * INCHES_TO_TICKS;   


    double currentX = totalPositionX * INCHES_TO_TICKS;
    double currentY = totalPositionY * INCHES_TO_TICKS;

    // calculating the triangle
    double h_mag_target = sqrt(pow(targetX - currentX, 2) + pow(targetY - currentY, 2)); // distance formula
    double theta_triangle_target = atan2(fabs(targetY - currentY), fabs(targetX - currentX));

    int startTime = pros::millis();
    int currentTime = pros::millis();
    int deltaTime = currentTime - startTime;

    double error_theta;
    int quadrant;

    // if (targetX > totalPositionX) {
    //     if (targetY > totalPositionY)
    //     {
    //         quadrant = 1;
    //         error_theta = 90 - theta_triangle;
    //     } else {
    //         quadrant = 4;
    //         error_theta = 90 + theta_triangle;
    //     }
    // } else {
    //     if (targetY > totalPositionY)
    //     {
    //         quadrant = 2;
    //         error_theta = theta_triangle - 90;
    //     } else {
    //         quadrant = 3;
    //         error_theta = -theta_triangle - 90;
    //     }
    // }

    
    // if(fabs(error_theta > 20)) {
    //     myPIDturn(error_theta, 1, .1, 1000);
    // }

    double kP_distance = gains[0];
    double kD_distance = gains[1];
    double kP_angle = gains[2];
    double kD_angle = gains[3];

    double prevError_h = 0;
    double prevError_angle = 0;




    switch (mode)
    {
    case 0: // motion profile
        /* code */
        break;
    case 1: // PID
        while ((fabs(targetY - currentY) > margins[1] * INCHES_TO_TICKS || fabs(targetX - currentX) > margins[0] * INCHES_TO_TICKS) && deltaTime < time)
        {

            pros::lcd::set_text(7, "I RAN ");
            
            currentX = totalPositionX * INCHES_TO_TICKS;
            currentY = totalPositionY * INCHES_TO_TICKS;


            currentTime = pros::millis();
            deltaTime = currentTime - startTime;



            // double error_x;
            // double error_y;

            // double power_x;
            // double power_y;

            // double prevError_x;
            // double prevError_y;

            // // X AXIS PID
            // if (fabs(targetX - currentX) > margins[0])
            // {
            //     error_x = targetX - totalPositionX;
            //     double deriv_x = error_x - prevError_x;
            //     prevError_x = error_x; 

            //     power_x = (error_x * kP_x) + (deriv_x * kD_x);
            // }
            
            // // Y AXIS PID
            // if (fabs(targetY - currentY) > margins[1])
            // {

            //     error_y = targetY - currentY;
            //     double deriv_y = error_y - prevError_y;
            //     prevError_y = error_y;
                
            //     power_y = (error_y * kP_y) + (deriv_y * kD_y);
            // }


            // distance and angle PID


                double h_mag_current;
                // double theta_triangle_current = atan2(fabs(targetY - currentY), fabs(targetX - currentX));

                double error_h;
                double error_angle;

                double deriv_h;
                double deriv_angle;

                double power_h;
                double power_tt;

            if (points[0]) {
                h_mag_current = sqrt(pow(currentX, 2) + pow(currentY, 2));
                // double theta_triangle_current = atan2(fabs(targetY - currentY), fabs(targetX - currentX));

                error_h = h_mag_target - h_mag_current;
                error_angle = (theta_triangle_target * (180/M_PI)) - inertial.get_rotation();

                deriv_h = error_h - prevError_h;
                deriv_angle = error_angle - prevError_angle;

                power_h = (error_h * kP_distance) + (deriv_h * kD_distance);
                power_tt = (error_angle * kP_angle) + (deriv_angle * kD_angle);

            } else {


                h_mag_current = currentY;
                // double theta_triangle_current = atan2(fabs(targetY - currentY), fabs(targetX - currentX));

                error_h = (points[1] * INCHES_TO_TICKS) - h_mag_current;
                deriv_h = error_h - prevError_h;

                power_h = (error_h * kP_distance) + (deriv_h * kD_distance);
                power_tt = 0;
            }

               

                prevError_h = error_h;
                prevError_angle = error_angle;

            pros::lcd::set_text(1, "h_mag_current:  " + std::to_string(h_mag_current));
            pros::lcd::set_text(2, "theta_triangle_current: " + std::to_string(theta_triangle_target));
            pros::lcd::set_text(3, "power_h: " + std::to_string(power_h));
            pros::lcd::set_text(4, "power_tt: " + std::to_string(power_tt));
            pros::lcd::set_text(5, "error_h: " + std::to_string(error_h) );
            pros::lcd::set_text(6, "error_tt: " + std::to_string(error_angle));
            
            set_drive_voltage(power_h + power_tt, power_h - power_tt);




            pros::delay(10);
        }
        set_drive_voltage(0,0);
        break;
    default:
        break;
    }
    
}



void move_to_point_PID(std::vector<double> points, std::vector<double> margins, std::vector<std::vector<double>> gains, double time) {


    // DATA FROM ODOM AND FROM FUNCTION PARAMETERS
    double curr_x = totalPositionX * INCHES_TO_TICKS;
    double curr_y = totalPositionY * INCHES_TO_TICKS;
    double curr_ang = inertial.get_rotation(); // IN DEGREES


    double t_x = points[0] * INCHES_TO_TICKS;
    double t_y = points[1] * INCHES_TO_TICKS;
    double t_ang = points[2]; // IN DEGREES

    double m_x = margins[0] * INCHES_TO_TICKS;
    double m_y = margins[1] * INCHES_TO_TICKS;
    double m_ang = margins[2]; // IN DEGREES


    // PID VARIABLES
    
    double err_x = t_x - curr_x;
    double err_y = t_y - curr_y;
    double err_ang = t_ang - curr_ang;

    double p_err_x, p_err_y, p_err_ang; // error 

    double power_x, power_y, power_position, power_heading; // voltage outputs 


    // PID CONSTANTS FOR EACH OF THE 3 CONTROLLERS
    double kP_x = gains[0][0];
    double kD_x = gains[0][1];

    double kP_y = gains[1][0];
    double kD_y = gains[1][1];

    double kP_ang = gains[2][0];
    double kD_ang = gains[2][1];

    // TIMER CONTROL
    double start_time = pros::millis();
    double current_time = pros::millis();
    double delta_time = 0;

  
    while ((fabs(err_x) > m_x || fabs(err_y) > m_y || fabs(err_ang) > m_ang) && delta_time < time)
    {
        // updating variabes
        current_time = pros::millis();
        delta_time = current_time - start_time;

        curr_x = totalPositionX * INCHES_TO_TICKS;
        curr_y = totalPositionY * INCHES_TO_TICKS;
        curr_ang = inertial.get_rotation();
        
        err_x = t_x - curr_x;
        err_y = t_y - curr_y;
        err_ang = t_ang - curr_ang;

        // position PD control
        power_x = (err_x * kP_x) + ((err_x - p_err_x) * kD_x);
        power_y = (err_y * kP_y) + ((err_y - p_err_y) * kD_y);


        // HEADING PID
        power_heading = (err_ang * kP_ang) + ((err_ang - p_err_ang) * kD_ang); 

        // final voltage output

        // forming arcs
        if (curr_x > t_x && fabs(err_x) > fabs(m_x)) {
            set_drive_voltage(power_y + power_heading, (power_y  / power_x) - power_heading);

            // debugging
            pros::lcd::print(1, "option 1");    

            pros::lcd::set_text(2, std::to_string(power_y + power_heading));    
            pros::lcd::set_text(3, std::to_string((power_y  / power_x) - power_heading));



        } else if (fabs(err_x) < fabs(m_x)) {
            set_drive_voltage(power_y + power_heading, power_y - power_heading);
            // debugging
            pros::lcd::print(1, "option 2");    
            // pros::lcd::set_text(2, std::to_string(power_y + power_heading));    
            // pros::lcd::set_text(3, std::to_string(power_y - power_heading));
pros::lcd::set_text(2, std::to_string(fabs(err_x)));            
pros::lcd::set_text(3, std::to_string(fabs(m_x)));


        } else if (curr_x < t_x && fabs(err_x) > fabs(m_x)) {
            set_drive_voltage((power_y  / power_x) + power_heading, power_y - power_heading);
            // debugging
            pros::lcd::print(1, "option 3");    
            pros::lcd::set_text(2, std::to_string(power_y));    
            pros::lcd::set_text(3, std::to_string(power_x));
            pros::lcd::set_text(4, std::to_string(power_heading));


        }



        // updating derivative p_err values
        p_err_x = err_x;
        p_err_y = err_y;
        p_err_ang = err_ang;

    }
    
    set_drive_voltage(0,0);

}
