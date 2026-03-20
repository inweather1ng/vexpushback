using namespace vex;

extern brain Brain;

//To set up a motor called LeftFront here, you'd use
//extern motor LeftFront;
extern controller Controller;

extern motor LeftFront;
extern motor LeftMiddle;
extern motor LeftBack;

extern motor RightFront;
extern motor RightMiddle;
extern motor RightBack;

extern motor IntakeBottom;
extern motor IntakeMiddle;
extern motor IntakeTop;
//Add your devices below, and don't forget to do the same in robot-config.cpp:

extern inertial Inertial;
extern distance Distance1;
extern led Scraper1;
extern led Scraper2;
extern led Descore;
extern led BallStop;

extern rotation VerticalOdom;
extern rotation HorizontalOdom;

void  vexcodeInit( void );