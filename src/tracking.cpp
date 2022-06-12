#include "main.h"

sPos gPosition;	

double totalPositionX = 0;
double totalPositionY = 0;
double totalPositionTheta = 0;

double deltaPositionX = 0;
double deltaPositionY = 0;

double prevLE = 0;
double prevRE = 0;
double prevSE = 0;



void positionTracking(void *ptr) {

	while (true)
	{


		double currentLE = leftEncoder.get_value();
		double currentRE = rightEncoder.get_value();
		double currentSE = midEncoder.get_value();


		double deltaL = (currentLE - prevLE) * TICKS_TO_INCHES; 
		double deltaR = (currentRE  - prevRE) * TICKS_TO_INCHES;
		double deltaS = (currentSE  - prevSE) * TICKS_TO_INCHES; 

		// Update the last values
		prevLE = currentLE;
		prevRE = currentRE;
		prevSE = currentSE;

		float deltaTheta = ((deltaL - deltaR) / (OFFSET_L + OFFSET_R)); 
		
		if (deltaTheta)
		{
			deltaPositionX = 2 * sin(deltaTheta/2) * ((deltaS/deltaTheta) + OFFSET_S);
			deltaPositionY = 2 * sin(deltaTheta/2) * ((deltaR/deltaTheta) + OFFSET_R);
		}
		else
		{
			deltaPositionX = deltaS;
			deltaPositionY = deltaR;
		}
		float thetaM = deltaTheta/2 + totalPositionTheta; 

		// cartesian (x,y) to polar (r, theta)
		double rPolar = sqrt(pow(deltaPositionX, 2) + pow(deltaPositionY, 2));
		double newThetaPolar = atan2(deltaPositionX, deltaPositionY) - thetaM;

		// polar back to cartesian
		// deltaPositionX = rPolar * sin(newThetaPolar);
		// deltaPositionY = rPolar * cos(newThetaPolar);

		totalPositionY += deltaPositionY * cos(thetaM);
		totalPositionX += deltaPositionY * sin(thetaM);

		totalPositionY += deltaPositionX * -sin(thetaM); // -sin(x) = sin(-x)
		totalPositionX += deltaPositionX * cos(thetaM); // cos(x) = cos(-x)

		

			// totalPositionX += deltaPositionX;
			// totalPositionY += deltaPositionY;

		totalPositionTheta += deltaTheta;

		// printing values to the brain screen for troubleshooting
		std::string xCoord = std::to_string(totalPositionX);
		std::string yCoord = std::to_string(totalPositionY);

		// std::cout << ( std::to_string(totalPositionTheta * (180/M_PI))) << std::endl;
		// std::cout << ( std::to_string(leftEncoder.get_value())) << std::endl;
		// std::cout << ( std::to_string(midEncoder.get_value())) << std::endl;
		// std::cout << ( std::to_string(rightEncoder.get_value())) << std::endl;
		// std::cout << ( "X: " + xCoord) << std::endl;
		// std::cout << ( "Y: " + yCoord) << std::endl;



		// pros::lcd::set_text(1, std::to_string(midEncoder.get_value()));
		// pros::lcd::set_text(2, std::to_string(leftEncoder.get_value()));
		// pros::lcd::set_text(3, std::to_string(rightEncoder.get_value()));
		// pros::lcd::set_text(4, std::to_string(totalPositionTheta * (180/M_PI)));

		pros::lcd::set_text(5, "X: " + xCoord);
		pros::lcd::set_text(6, "Y: " + yCoord);

		pros::delay(10);
	}
}

	