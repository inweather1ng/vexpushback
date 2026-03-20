#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen.
brain  Brain;

//The motor constructor takes motors as (port, ratio, reversed), so for example
//motor LeftFront = motor(PORT1, ratio6_1, false);
controller Controller = controller(primary);

motor RightFront = motor(PORT10, ratio6_1, false);
motor RightMiddle = motor(PORT9, ratio6_1, true);
motor RightBack = motor(PORT8, ratio6_1, false);// in 9 right now

motor LeftFront = motor(PORT7, ratio6_1, true);
motor LeftMiddle = motor(PORT6, ratio6_1, false);
motor LeftBack = motor(PORT5, ratio6_1, true); 

motor IntakeBottom = motor(PORT1, ratio6_1, false);
motor IntakeTop = motor(PORT2, ratio18_1, false);

led Scraper1(Brain.ThreeWirePort.H);
led Scraper2(Brain.ThreeWirePort.G);
led Descore(Brain.ThreeWirePort.F);
led BallStop(Brain.ThreeWirePort.E);

led WheelPiston(Brain.ThreeWirePort.C);
//Add your devices below, and don't forget to do the same in robot-config.h:

inertial Inertial = inertial(PORT19);
distance Distance1 = distance(PORT20);
rotation VerticalOdom = rotation(PORT4, true);
rotation HorizontalOdom = rotation(PORT3, false);


void vexcodeInit( void ) {
  // nothing to initialize
}
