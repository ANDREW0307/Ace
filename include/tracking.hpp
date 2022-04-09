#include "main.h"

// tracking wheel diameters in inches 
#define TRACKING_WHEEL_DIAMETER 2.8

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
#define SIDE_DISTANCE_LEFT (7.0 + 1.0/16.0) / 2.07
#define SIDE_DISTANCE_RIGHT (7.0 + 1.0/16.0) / 2.07
#define FORWARDS_DISTANCE_BACK 2

// the number of ticks per rotations of the tracking wheel - analogous to degrees on a vex quadrature encoder
#define TICKS_PER_ROTATION_TRACKING 360.0

// constants used in the conversion from ticks to inches and vice versa
#define TICKS_TO_INCHES_CONSTANT_LR (TRACKING_WHEEL_DIAMETER * M_PI) / TICKS_PER_ROTATION_TRACKING // left & right
#define TICKS_TO_INCHES_CONSTANT_S (TRACKING_WHEEL_DIAMETER * M_PI) / TICKS_PER_ROTATION_TRACKING // back
#define INCHES_TO_TICKS_CONSTANT_LRS (TICKS_PER_ROTATION_TRACKING) / (TRACKING_WHEEL_DIAMETER * M_PI)

// position values tp be used by various tasks
typedef struct _pos
{
	std::atomic<double> theta;
	std::atomic<double> y;
	std::atomic<double> x;

	std::atomic<double> leftLst;
	std::atomic<double> rightLst;
	std::atomic<double> backLst;

} sPos; 



void positionTracking(void* ptr);

extern sPos gPosition;
