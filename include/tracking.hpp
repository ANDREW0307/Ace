#include "main.h"

// tracking wheel diameters in inches 
#define TRACKING_WHEEL_DIAMETER 2.738

//                                           +-+  Center of rotation (CoR)
//                                           |
//                                           +----------+ Length from CoR to back wheel
//                                           |          |     FORWARDS_DISTANCE_BACK
//                                  ===      |      === |
//                                   +       v       +  |
//                                  ++---------------++ |
//     SIDE_DISTANCE_LEFT/RIGHT     |                 | | 
//                         +--->    |       ---       | V
//                         |        |                 |
//                         +--->    |        x        |+|  <-- back tracking wheel
//                                  |                 |
//                                  |       ---       |
//                                  ++---------------++
//                                   +       ^       +
//                                  ===      |      ===
//                                           |                     
//                               left / right tracking wheels

// the distances between the tracking wheels and tracking center of the robot in inches (as shown above)
#define OFFSET_L ((8.0 + 6.7/16.0) / 2.0)
#define OFFSET_R ((8.0 + 6.7/16.0) / 2.0)
#define OFFSET_S 0.0

// the number of ticks per rotations of the tracking wheel - analogous to degrees on a vex quadrature encoder
#define TICKS_PER_ROTATION_TRACKING 360.0

// constants used in the conversion from ticks to inches and vice versa
#define TICKS_TO_INCHES ((TRACKING_WHEEL_DIAMETER * M_PI) / TICKS_PER_ROTATION_TRACKING)

#define INCHES_TO_TICKS (TICKS_PER_ROTATION_TRACKING) / (TRACKING_WHEEL_DIAMETER * M_PI)

// position values tp be used by various tasks
typedef struct _pos
{
	// std::atomic<double> theta;
	// std::atomic<double> y;
	// std::atomic<double> x;

	// std::atomic<double> leftLst;
	// std::atomic<double> rightLst;
	// std::atomic<double> backLst;

	float theta;
	float y;
	float x;

	float leftLst;
	float rightLst;
	float backLst;

} sPos; 



void positionTracking(void* ptr);


extern double totalPositionX;
extern double totalPositionY;
extern double totalPositionTheta;

extern sPos gPosition;
