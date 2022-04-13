#include "main.h"


sPos gPosition;
void positionTracking(void *ptr) {

	while (true)
	{
			
		double L = (leftEncoder.get_value() - gPosition.leftLst) * TICKS_TO_INCHES_CONSTANT_LR; // The amount the left side of the robot moved
		double R = (rightEncoder.get_value()  - gPosition.rightLst) * TICKS_TO_INCHES_CONSTANT_LR; // The amount the right side of the robot moved
		double S = (midEncoder.get_value()  - gPosition.backLst) * TICKS_TO_INCHES_CONSTANT_S; // The amount the back side of the robot moved

		// Update the last values
		gPosition.leftLst = leftEncoder.get_value();
		gPosition.rightLst = rightEncoder.get_value();
		gPosition.backLst = midEncoder.get_value();

		double h; // The hypotenuse of the triangle formed by the middle of the robot on the starting position and ending position and the middle of the circle it travels around
		double i; // Half on the angle that I've traveled
		double h2; // The same as h but using the back instead of the side wheels
		double a = ((L - R) / (SIDE_DISTANCE_LEFT + SIDE_DISTANCE_RIGHT)); // The angle that I've traveled
		// double a = ((-inertial.get_rotation() * (M_PI/180)) + ((L - R) / (SIDE_DISTANCE_LEFT + SIDE_DISTANCE_RIGHT))) / 2; // The angle that I've traveled
		// double a = (-inertial.get_rotation() * (M_PI/180)); // The angle that I've traveled
		
		if (a)
		{
			double r = R / a; // The radius of the circle the robot travel's around with the right side of the robot
			i = a / 2.0;
			double sinI = sin(i);
			h = ((r + SIDE_DISTANCE_RIGHT) * sinI) * 2.0;

			double r2 = S / a; // The radius of the circle the robot travel's around with the back of the robot
			h2 = ((r2 + FORWARDS_DISTANCE_BACK) * sinI) * 2.0;
		}
		else
		{
			h = R;
			i = 0;

			h2 = S;
		}
		double p = i + gPosition.theta; // The global ending angle of the robot
		double cosP = cos(p);
		double sinP = sin(p);

		// Update the global position - note: std::atomic doesn't recognize "+="" and "-=", so I have to write it this way
		gPosition.y = gPosition.y + (h * cosP);
		gPosition.x = gPosition.x + (h * sinP);

		gPosition.y = gPosition.y + (h2 * -sinP); // -sin(x) = sin(-x)
		gPosition.x = gPosition.x + (h2 * cosP); // cos(x) = cos(-x)

		gPosition.theta = gPosition.theta + a;




		// printing values to the brain screen for troubleshooting
		std::string xCoord = std::to_string(gPosition.x);
		std::string yCoord = std::to_string(gPosition.y);

		// pros::lcd::set_text(2, std::to_string(gPosition.theta * (180/M_PI)));
		// pros::lcd::set_text(3, std::to_string(leftEncoder.get_value()));
		// pros::lcd::set_text(4, std::to_string(midEncoder.get_value()));
		// pros::lcd::set_text(5, std::to_string(rightEncoder.get_value()));
		// pros::lcd::set_text(6, "X: " + xCoord);
		// pros::lcd::set_text(7, "Y: " + yCoord);

		// 20 ms refresh rate 
		pros::delay(10);
	}
}

	