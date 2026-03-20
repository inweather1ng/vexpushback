#include "vex.h"

/**
 * Resets the constants for auton movement.
 * Modify these to change the default behavior of functions like
 * drive_distance(). For explanations of the difference between
 * drive, heading, turning, and swinging, as well as the PID and
 * exit conditions, check the docs.
 */


  

void default_constants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.set_heading_constants(4, .4, 0, 1, 0);
  chassis.set_turn_constants(6, .4, .03, 3, 15);
  chassis.set_swing_constants(6, .3, .001, 2, 15);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(1.5, 300, 3000);
  chassis.set_turn_exit_conditions(1, 300, 3000);
  chassis.set_swing_exit_conditions(1, 300, 3000);
}

/**
 * Sets constants to be more effective for odom movements.
 * For functions like drive_to_point(), it's often better to have
 * a slower max_voltage and greater settle_error than you would otherwise.
 */

void odom_constants(){
  default_constants();
  chassis.heading_max_voltage = 10;
  chassis.drive_max_voltage = 8;
  chassis.drive_settle_error = 3;
  chassis.boomerang_lead = .5;
  chassis.drive_min_voltage = 0;
}

/**
 * The expected behavior is to return to the start position.
 */


/**
 * The expected behavior is to return to the start angle, after making a complete turn.
 */



/**
 * Should swing in a fun S shape.
 */



/**
 * A little of this, a little of that; it should end roughly where it started.
 */



/**
 * Doesn't drive the robot, but just prints coordinates to the Brain screen 
 * so you can check if they are accurate to life. Push the robot around and
 * see if the coordinates increase like you'd expect.
 */

void odom_test(){
  chassis.set_coordinates(0, 0, 0);
  while(1){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5,20, "X: %f", chassis.get_X_position());
    Brain.Screen.printAt(5,40, "Y: %f", chassis.get_Y_position());
    Brain.Screen.printAt(5,60, "Heading: %f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5,80, "ForwardTracker: %f", chassis.get_ForwardTracker_position());
    Brain.Screen.printAt(5,100, "SidewaysTracker: %f", chassis.get_SidewaysTracker_position());
    task::sleep(20);
  }
}


/**
 * prints coordinates to the Brain screen 
 * so you can check if they are accurate to life. Push the robot around and
 * see if the coordinates increase like you'd expect.
 */

void display_screen(){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5,20, "X: %f", chassis.get_X_position());
    Brain.Screen.printAt(5,40, "Y: %f", chassis.get_Y_position());
    Brain.Screen.printAt(5,60, "Heading: %f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5,80, "ForwardTracker: %f", chassis.get_ForwardTracker_position());
    Brain.Screen.printAt(5,100, "SidewaysTracker: %f", chassis.get_SidewaysTracker_position());
    Brain.Screen.printAt(5,120, "Avg Distance: %f", (chassis.get_left_position_in()+chassis.get_right_position_in())/2.0);
    Brain.Screen.printAt(5,140, "Left Deg: %f  :: Right Deg: %f", chassis.DriveL.position(deg),chassis.DriveR.position(deg));

    //Brain.Screen.printAt(5,140, "Y: %f", chassis.get_Y_position());
   
    task::sleep(500);
  
}

/**
 * Should end in the same place it began, but the second movement
 * will be curved while the first is straight.
 */



/**
 * Drives in a square while making a full turn in the process. Should
 * end where it started.
 */
double distance_reading = Distance1.objectDistance(inches);
void detect(double reading){
  if(distance_reading < reading){
    chassis.drive_distance(reading-distance_reading);
  }
  else if(distance_reading > reading){
    chassis.drive_distance(-distance_reading + reading);
  }
  else{
    chassis.drive_distance(0);
  }



}
void threeloaders(){
  Scraper1.set(true);
  Scraper2.set(true);
  chassis.drive_distance(20);
  chassis.turn_to_angle(30);
  chassis.drive_distance(25);
  chassis.right_swing_to_angle(0);
  chassis.drive_distance(30);

  chassis.turn_to_angle(90);
  Scraper1.set(true);
  Scraper2.set(true);
}
void longroutecoordsskills(){
 
// --- JARLIB Setup ---
// Set the absolute starting pose: (X=88", Y=20", Angle=0 degrees)
chassis.set_coordinates(88, 20, 0); 

display_screen();
wait(1000, msec);

// --- Initial Setup ---
IntakeTop.setVelocity(100, percent);
IntakeBottom.setVelocity(100, percent);
BallStop.set(false);
Scraper1.set(false);
Scraper2.set(false);

// ----------------------------------------------------------------
// ## Phase 1: Bottom Towers (with Wall Squaring and Load/Unload)
// ----------------------------------------------------------------

// P1.1: Drive to Bottom-Right Tower (Target 120, 24)
chassis.drive_to_pose(120, 24, 0); 

// P1.2: UNLOAD/LOAD ACTION at Bottom-Right Tower
IntakeBottom.spin(forward); // Assuming forward is the load/unload direction
wait(1000, msec);
IntakeBottom.stop();

// P1.3: Drive to Squaring Y-Coordinate
chassis.drive_to_pose(10, 24, 0); 
// P1.4: Turn to Left Wall
chassis.drive_to_pose(10, 24, 270); 

// P1.5: WALL SQUARING ACTION: Drive against the X=0 wall
chassis.drive_to_pose(0, 24, 270);
// P1.6: Correct Odometry Drift
chassis.set_coordinates(0, 24, 270); 

// P1.7: Drive to Bottom-Left Tower (Target 24, 24)
chassis.drive_to_pose(24, 24, 270); 

// P1.8: UNLOAD/LOAD ACTION at Bottom-Left Tower
IntakeBottom.spin(forward); // Use the same action as the right tower
wait(1000, msec); 
IntakeBottom.stop();
Scraper1.set(false); 
Scraper2.set(false);

// ----------------------------------------------------------------
// ## Phase 1: Bottom Towers (with Wall Squaring)
// ----------------------------------------------------------------

// P1.1: Drive to Bottom-Right Tower
chassis.drive_to_pose(120, 24, 0); 
// P1.2: Intake 1 
Scraper1.set(true); 
IntakeBottom.spin(reverse); 
wait(1000, msec);

// P1.3: Drive to Squaring Y-Coordinate
chassis.drive_to_pose(10, 24, 0); 
// P1.4: Turn to Left Wall
chassis.drive_to_pose(10, 24, 270); 

// P1.5: WALL SQUARING ACTION: Drive against the X=0 wall
chassis.drive_to_pose(0, 24, 270);
// P1.6: Correct Odometry Drift
chassis.set_coordinates(0, 24, 270); 

// P1.7: Drive to Bottom-Left Tower
chassis.drive_to_pose(24, 24, 270); 
// P1.8: Intake 2 
Scraper2.set(true); 
wait(1000, msec); 
Scraper1.set(false); 
Scraper2.set(false);


// ----------------------------------------------------------------
// ## Phase 2: Top-Left Tower & Unload (with Wall Squaring)
// ----------------------------------------------------------------

// P2.1: Drive up to a Squaring Y-Coordinate
chassis.drive_to_pose(24, 100, 270); 
// P2.2: WALL SQUARING ACTION: Drive against the X=0 wall
chassis.drive_to_pose(0, 100, 270); 
// P2.3: Correct Odometry Drift
chassis.set_coordinates(0, 100, 270); 

// P2.4: Drive to Top-Left Tower Access
chassis.drive_to_pose(24, 120, 0); 
// P2.5: Intake 3: Collect/Empty Top-Left Tower.
IntakeTop.spin(reverse);
IntakeBottom.spin(reverse);
wait(1500, msec);
IntakeTop.stop();
IntakeBottom.stop();

// P2.6: Reverse out from Tower to Loader Unload Zone
chassis.drive_to_pose(24, 100, 0); 
// P2.7: Turn to face the goal/loader (facing left, 270 degrees)
chassis.drive_to_pose(24, 100, 270); 

// P2.8: Drive/Push into the Loader/Goal on the X=24 line.
chassis.drive_to_pose(10, 100, 270); 
// P2.9: Eject/Load into Goal.
IntakeTop.spin(forward); 
IntakeBottom.spin(forward); 
wait(2000, msec);
IntakeTop.stop();
IntakeBottom.stop();

// P2.10: Reverse away from the goal.
chassis.drive_to_pose(30, 100, 270); 


// ----------------------------------------------------------------
// ## Phase 3: Park
// ----------------------------------------------------------------

// P3.1: Turn down field.
chassis.drive_to_pose(30, 100, 180); 
// P3.2: Drive to a clear parking entry point.
chassis.drive_to_pose(80, 24, 180); 
// P3.3: Drive into the parking zone.
chassis.drive_to_pose(80, 10, 180);
}
distance2_reading = Distance2.object_distance(inches);
distance3_reading = Distance3.object_distance(inches);
void anglereset(double favoredangle){
  double error = cout << std::atan()
  if (distance2_reading > distance3_reading){
    chassis.turn_to_angle

  }
  


}
void riskroute(){
IntakeBottom.setVelocity(100, percent);  
  IntakeTop.setVelocity(100, percent);
  Descore.set(true);

  // drive towards loader, turn, put scraper down, drive into loader
  chassis.set_drive_exit_conditions(1.5, 300, 2000);
  chassis.drive_distance(34, 0);
  chassis.set_turn_exit_conditions(1, 300, 1000);
  chassis.turn_to_angle(90);
  Scraper1.set(true);
  Scraper2.set(true);
  IntakeBottom.spin(reverse);
  chassis.set_drive_exit_conditions(1.5, 300, 2000);
  wait(200, msec);
  chassis.set_drive_constants(4, 1.5, 0, 10, 0);
  chassis.drive_distance(11.25, 90);
  //chassis.turn_to_angle(85);
  wait(1750, msec);

  // back away and drive to other side of field
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.drive_distance(-5);
  IntakeBottom.stop();
  IntakeTop.stop();
  Scraper1.set(false);
  Scraper2.set(false);
  chassis.set_turn_exit_conditions(1, 300, 1000);
  chassis.turn_to_angle(135);
  chassis.set_drive_exit_conditions(1.5, 300, 2000);
  chassis.drive_distance(-9, 135); 
  chassis.set_turn_exit_conditions(1, 300, 500);
  chassis.left_swing_to_angle(90);
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  chassis.drive_distance(-68, 85);

  // line up with long goal and score
  chassis.set_turn_exit_conditions(1, 300, 500);
  //chassis.turn_to_angle(0);
  //chassis.right_swing_to_angle(-45);
  chassis.left_swing_to_angle(0);
  chassis.set_drive_exit_conditions(1.5, 300, 2000);
  chassis.drive_distance(-9, 0);//prev -10
  chassis.turn_to_angle(-90);
  chassis.set_drive_exit_conditions(1.5, 300, 1000);
  chassis.drive_distance(-18, -90);
  IntakeBottom.spin(reverse);
  IntakeTop.spin(reverse);
  chassis.turn_to_angle(-90);
  chassis.drive_distance(-2, -90);
  wait(1750, msec);
  
  //drive into second loader, get blocks, score on long goal
  IntakeTop.stop(); 
  Scraper1.set(true);
  Scraper2.set(true);
  wait(200, msec);
  chassis.set_drive_constants(4, 1.5, 0, 10, 0);
  chassis.set_drive_exit_conditions(1.5, 300, 1500);
  chassis.drive_distance(28.75, -90);
  wait(1500, msec);
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.drive_distance(-31, -90);
  IntakeTop.spin(reverse);
  wait(1500, msec);
  chassis.drive_distance(-2, -90);
  wait(1700, msec);
  //chassis.turn_to_angle(-90);

  //move accross field, line up with third loader, get blocks
  Scraper1.set(false);
  Scraper2.set(false);
  IntakeTop.stop(); 
  chassis.set_drive_exit_conditions(1.5, 300, 2000);
  chassis.drive_distance(18, -90);
  chassis.set_turn_exit_conditions(1, 300, 1000);
  chassis.turn_to_angle(-180);
  chassis.set_drive_exit_conditions(1.5, 300, 3500);
  chassis.drive_distance(97, -185);
  chassis.turn_to_angle(-90);
  Scraper1.set(true);
  Scraper2.set(true);
  chassis.set_drive_constants(4, 1.5, 0, 10, 0);
  IntakeBottom.spin(reverse);
  chassis.drive_distance(15.5);
  wait(1400, msec);
  chassis.turn_to_angle(-90);

  // back away and move to other side of field
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.drive_distance(-5);
  IntakeTop.stop();
  IntakeBottom.stop();
  Scraper1.set(false);
  Scraper2.set(false);
  chassis.turn_to_angle(-45);
  chassis.set_drive_exit_conditions(1.5, 300, 1000);
  chassis.drive_distance(-11, -45);
  chassis.set_turn_exit_conditions(1, 300, 750);
  chassis.left_swing_to_angle(-90);
  chassis.set_drive_exit_conditions(1.5, 300, 2000);
  chassis.drive_distance(-70, -85);

  //line up with long goal and score
  chassis.left_swing_to_angle(-180);
  chassis.set_drive_exit_conditions(1.5, 300, 1000);
  chassis.drive_distance(-8, -180);
  chassis.turn_to_angle(-270);
  chassis.drive_distance(-18, -270);
  IntakeBottom.spin(reverse);
  IntakeTop.spin(reverse);
  wait(1900, msec);
  IntakeTop.stop();

  //get fourth loader and score
  Scraper1.set(true);
  Scraper2.set(true);
  chassis.set_drive_constants(4, 1.5, 0, 10, 0);
  chassis.set_drive_exit_conditions(1.5, 300, 1500);
  chassis.drive_distance(28.75, -270);
  wait(1500, msec);
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.set_drive_exit_conditions(1.5, 300, 1000);
  chassis.drive_distance(-29.25, -270);
  IntakeTop.spin(reverse);
  wait(1900, msec);

  //park
  Scraper1.set(false);
  Scraper2.set(false);
  chassis.drive_distance(20, -270);
  chassis.turn_to_angle(45);
  chassis.drive_distance(17);
  chassis.right_swing_to_angle(10);
  chassis.drive_distance(9, 10);
  Scraper1.set(true);
  Scraper2.set(true);
  IntakeTop.spin(reverse);
  IntakeBottom.spin(reverse);
  chassis.set_drive_exit_conditions(1.5, 300, 750);
  chassis.drive_distance(33, 5);
  chassis.set_drive_exit_conditions(1.5,300,1000);
  chassis.drive_distance(-5);
  Scraper1.set(false);
  Scraper2.set(false);

  












  


  

}
void newroute(){
  // get block from teh middle
  IntakeBottom.setVelocity(100, percent);
  IntakeTop.setVelocity(100, percent);
  chassis.set_drive_exit_conditions(1,300,1500);
  chassis.drive_distance(9);
  chassis.turn_to_angle(-90);
  chassis.set_drive_exit_conditions(1, 300, 3000);
  chassis.drive_distance(39);
  chassis.set_drive_exit_conditions(1,300,1500);
  // score blocks in the upper middle goal
  chassis.turn_to_angle(45);
  IntakeBottom.spin(reverse);
  chassis.drive_distance(13);
  IntakeBottom.stop();
  chassis.turn_to_angle(212.5);
  IntakeBottom.setVelocity(50, percent);
  IntakeTop.setVelocity(50, percent);

  chassis.drive_max_voltage = 4;
  chassis.drive_distance(-30);
  chassis.drive_max_voltage = 8;

  BallStop.set(true);
  IntakeBottom.spin(reverse);
  IntakeTop.spin(reverse);
  wait(1000,msec);
  IntakeBottom.stop();
  IntakeTop.stop();
  chassis.turn_to_angle(225);
  chassis.drive_max_voltage = 8;
  chassis.set_drive_exit_conditions(1,300,4000);
  chassis.drive_distance(49);
  chassis.set_drive_exit_conditions(1,300,1500);
  chassis.turn_to_angle(180);
  chassis.drive_distance(5,180);
  BallStop.set(false);
  wait(875, msec);
  chassis.turn_to_angle(180);
  wait(50, msec);
  Scraper1.set(true);
  Scraper2.set(true);
  wait(50, msec);
  chassis.drive_distance(-5);
  Scraper1.set(false);
  Scraper2.set(false);
  chassis.turn_to_angle(135);
  chassis.drive_distance(-12);
  chassis.turn_to_angle(180);
  chassis.set_drive_exit_conditions(1,300,5000);
  chassis.drive_distance(-68);
  chassis.turn_to_angle(-90);
  chassis.set_drive_exit_conditions(1,300,2500);
  chassis.drive_distance(-7);
  chassis.turn_to_angle(0);
  chassis.drive_distance(-18);
  IntakeBottom.spin(reverse);
  IntakeTop.spin(reverse);
  wait(2000, msec);
  IntakeTop.stop();
  chassis.turn_to_angle(90);
  wait(50, msec);
  Scraper1.set(true);
  Scraper2.set(true);
  wait(50, msec);
  chassis.drive_distance(29.25,0);
  wait(875, msec);
  chassis.drive_distance(-29.25,0);
  IntakeTop.spin(reverse);
  wait(2000, msec);
  Scraper1.set(false);
  Scraper2.set(false);
  IntakeTop.stop();
  IntakeBottom.stop();
  chassis.set_drive_exit_conditions(1,300,1000);
  chassis.drive_distance(17);
  chassis.turn_to_angle(80);
  chassis.set_drive_exit_conditions(1,300,750);
  chassis.drive_distance(9,80);
  IntakeBottom.spin(reverse);
  chassis.set_drive_exit_conditions(1,300,2000);
  chassis.drive_max_voltage = 6;
  chassis.drive_distance(10);
  wait(100, msec);
  chassis.drive_distance(10);
  wait(100, msec);
  IntakeBottom.stop();
  chassis.drive_distance(35);
  chassis.turn_to_angle(180);
  chassis.drive_distance(17);
  chassis.turn_to_angle(-90);
  chassis.turn_to_angle(180);
  chassis.drive_distance(24);
  chassis.turn_to_angle(45);
  chassis.drive_distance(-30);
  BallStop.set(true);
  IntakeBottom.spin(reverse);
  IntakeTop.spin(reverse);
  wait(2000, msec);
  IntakeTop.stop();
  IntakeBottom.stop();
  chassis.set_drive_exit_conditions(1,300,4000);
  chassis.drive_distance(48);
  chassis.set_drive_exit_conditions(1,300,3000);
  chassis.turn_to_angle(0);
  Scraper1.set(true);
  Scraper2.set(true);
  
  wait(555, msec);
  IntakeBottom.spin(reverse);
  chassis.drive_distance(5);
  wait(875, msec);
  IntakeBottom.stop();
  BallStop.set(false);
  chassis.drive_distance(-5);
  Scraper1.set(false);
  Scraper2.set(false);
  chassis.turn_to_angle(135);
  chassis.set_drive_exit_conditions(1,300,4000);
  chassis.drive_distance(-12, 135);
  chassis.turn_to_angle(180);
  chassis.drive_distance(68);
  chassis.set_drive_exit_conditions(1,300,2500);
  chassis.turn_to_angle(90);
  chassis.drive_distance(-7);
  chassis.turn_to_angle(180);
  chassis.drive_distance(-18);
  IntakeBottom.spin(reverse);
  IntakeTop.spin(reverse);
  wait(2000, msec);
  IntakeTop.stop();
  IntakeBottom.stop();
  chassis.turn_to_angle(0);
  Scraper1.set(true);
  Scraper2.set(true);
  IntakeBottom.spin(reverse);
  chassis.drive_distance(29.25);
  wait(1500, msec);
  chassis.drive_distance(-29.25);
  IntakeTop.spin(reverse);
  wait(1500, msec);
  chassis.drive_distance(17);
  chassis.turn_to_angle(-100);
  chassis.drive_distance(9,10);
  IntakeBottom.spin(reverse);
  IntakeTop.spin(reverse);
  Scraper1.set(true);
  Scraper2.set(true);
  chassis.drive_distance(43);


  







  













  

}
/* 
void points(){
  chassis.drive_distance(32.5);
  wait(100,msec);
  chassis.turn_to_angle(90);
  Scraper1.set(true);
  Scraper2.set(true);
  IntakeBottom.spin(reverse);
  chassis.drive_distance(10.75);
  wait(1500,msec);
  chassis.drive_distance(-5);
  Scraper1.set(false);
  Scraper2.set(false);
  IntakeBottom.stop();
  chassis.turn_to_angle(-45);
  chassis.drive_distance(24);
  chassis.turn_to_angle(-90);
  chassis.drive_distance(72);
  chassis.turn_to_angle(-45);
  chassis.drive_distance(-30);
  chassis.turn_to_angle(-90);
  IntakeBottom.spin(reverse);
  IntakeTop.spin(reverse);
  wait(3000,msec);
  IntakeTop.stop();
  Scraper1.set(true);
  Scraper2.set(true);
  chassis.drive_distance(29.25);
  wait(1500,msec);
  chassis.drive_distance(-5);
  Scraper1.set(false);
  Scraper2.set(false);
  chassis.drive_distance(-24.5);
  Intake_top.spin(reverse);
  wait(3000,msec);
  Intake_Top.stop();
  Intake_Bottom.stop();
  chassis.drive_distance(6);
  chassis.turn_to_angle(180);
  chassis.drive_distance(24);
  chassis.turn_to_angle(90);
  Intake_Bottom.spin(reverse);
  chassis.drive_distance(6);
  wait(500,msec);
  chassis.turn_to_angle(315);
  BallBlock.set(false);
  chassis.drive_distance(-27);
  IntakeTop.spin(reverse);
  wait(200,msec);
  BallBlock.set(true);
  IntakeTop.stop();
  chassis.drive_distance(24);
  chassis.turn_to_angle(180);
  chassis.drive_distance(46);
  wait(200,msec);

  chassis.turn_to_angle(225);
  chassis.drive_distance(-27);
  IntakeTop.spin(reverse);
  
    




  
  

} */
//void anglereset(float angle,float distancebetweensensors,float heading){
  //if frontleftdistance>
  //turn_to_angle(std::atan((frontleftdistance-frontrightdistance)/distancebetweensensors)+float heading)
  



void coordsskills(){
  
  IntakeBottom.setVelocity(100, percent);  
  IntakeTop.setVelocity(100, percent);
  Descore.set(true);

  // drive towards loader, turn, put scraper down, drive into loader
  chassis.set_drive_exit_conditions(1.5, 300, 2000);
  chassis.drive_distance(34, 0);
  chassis.set_turn_exit_conditions(1, 300, 1000);
  chassis.turn_to_angle(90);
  Scraper1.set(true);
  Scraper2.set(true);
  IntakeBottom.spin(reverse);
  chassis.set_drive_exit_conditions(1.5, 300, 2000);
  wait(200, msec);
  chassis.set_drive_constants(4, 1.5, 0, 10, 0);
  chassis.drive_distance(11.25, 90);
  //chassis.turn_to_angle(85);
  wait(1750, msec);

  // back away and drive to other side of field
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.drive_distance(-5);
  IntakeBottom.stop();
  IntakeTop.stop();
  Scraper1.set(false);
  Scraper2.set(false);
  chassis.set_turn_exit_conditions(1, 300, 1000);
  chassis.turn_to_angle(135);
  chassis.set_drive_exit_conditions(1.5, 300, 2000);
  
  chassis.drive_distance(-9, 135); 
  chassis.set_turn_exit_conditions(1, 300, 500);
  chassis.left_swing_to_angle(90);
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  chassis.drive_distance(-68, 85);

  // line up with long goal and score
  chassis.set_turn_exit_conditions(1, 300, 500);
  //chassis.turn_to_angle(0);
  //chassis.right_swing_to_angle(-45);
  chassis.left_swing_to_angle(0);
  chassis.set_drive_exit_conditions(1.5, 300, 2000);
  chassis.drive_distance(-9, 0);//prev -10
  chassis.turn_to_angle(-90);
  chassis.set_drive_exit_conditions(1.5, 300, 1000);
  chassis.drive_distance(-18, -90);
  IntakeBottom.spin(reverse);
  IntakeTop.spin(reverse);
  chassis.turn_to_angle(-90);
  chassis.drive_distance(-2, -90);
  wait(1750, msec);
  
  //drive into second loader, get blocks, score on long goal
  IntakeTop.stop(); 
  Scraper1.set(true);
  Scraper2.set(true);
  wait(200, msec);
  chassis.set_drive_constants(4, 1.5, 0, 10, 0);
  chassis.set_drive_exit_conditions(1.5, 300, 1500);
  chassis.drive_distance(28.75, -90);
  wait(1500, msec);
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.drive_distance(-31, -90);
  IntakeTop.spin(reverse);
  wait(1500, msec);
  chassis.drive_distance(-2, -90);
  wait(1700, msec);
  //chassis.turn_to_angle(-90);

  //move accross field, line up with third loader, get blocks
  Scraper1.set(false);
  Scraper2.set(false);
  IntakeTop.stop(); 
  chassis.set_drive_exit_conditions(1.5, 300, 2000);
  chassis.drive_distance(18, -90);
  chassis.set_turn_exit_conditions(1, 300, 1000);
  chassis.turn_to_angle(-180);
  chassis.set_drive_exit_conditions(1.5, 300, 3500);
  chassis.drive_distance(97, -185);
  chassis.turn_to_angle(-90);
  Scraper1.set(true);
  Scraper2.set(true);
  chassis.set_drive_constants(4, 1.5, 0, 10, 0);
  IntakeBottom.spin(reverse);
  chassis.drive_distance(15.5);
  wait(1400, msec);
  chassis.turn_to_angle(-90);

  // back away and move to other side of field
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.drive_distance(-5);
  IntakeTop.stop();
  IntakeBottom.stop();
  Scraper1.set(false);
  Scraper2.set(false);
  chassis.turn_to_angle(-45);
  chassis.set_drive_exit_conditions(1.5, 300, 1000);
  chassis.drive_distance(-11, -45);
  chassis.set_turn_exit_conditions(1, 300, 750);
  chassis.left_swing_to_angle(-90);
  chassis.set_drive_exit_conditions(1.5, 300, 2000);
  chassis.drive_distance(-70, -85);

  //line up with long goal and score
  chassis.left_swing_to_angle(-180);
  chassis.set_drive_exit_conditions(1.5, 300, 1000);
  chassis.drive_distance(-8, -180);
  chassis.turn_to_angle(-270);
  chassis.drive_distance(-18, -270);
  IntakeBottom.spin(reverse);
  IntakeTop.spin(reverse);
  wait(1900, msec);
  IntakeTop.stop();

  //get fourth loader and score
  Scraper1.set(true);
  Scraper2.set(true);
  chassis.set_drive_constants(4, 1.5, 0, 10, 0);
  chassis.set_drive_exit_conditions(1.5, 300, 1500);
  chassis.drive_distance(28.75, -270);
  wait(1500, msec);
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.set_drive_exit_conditions(1.5, 300, 1000);
  chassis.drive_distance(-29.25, -270);
  IntakeTop.spin(reverse);
  wait(1900, msec);

  //park
  Scraper1.set(false);
  Scraper2.set(false);
  chassis.drive_distance(20, -270);
  chassis.turn_to_angle(45);
  chassis.drive_distance(17);
  chassis.right_swing_to_angle(10);
  chassis.drive_distance(9, 10);
  Scraper1.set(true);
  Scraper2.set(true);
  IntakeTop.spin(reverse);
  IntakeBottom.spin(reverse);
  chassis.set_drive_exit_conditions(1.5, 300, 750);
  chassis.drive_distance(33, 5);
  chassis.drive_distance(-5);
  Scraper1.set(false);
  Scraper2.set(false);


  // IntakeBottom.spin(reverse);
  // IntakeTop.spin(reverse);
  // wait(2500,msec);.spin(reverse)
  // IntakeBottom.stop();
  // IntakeTop.stop();
  // chassis.drive_to_pose(120,30,90);
  // chassis.turn_to_angle(180);
  // chassis.drive_to_pose(24,30,180);
  // chassis.turn_to_angle(90);
  // Scraper1.set(true);
  // Scraper2.set(true);
  // chassis.drive_to_pose(24,9,90);
  // IntakeBottom.spin(reverse);
  // wait(2500,msec);
  // IntakeBottom.stop();
  // chassis.drive_to_pose(24,24,90);
  // Scraper1.set(false);
  // Scraper2.set(false);
  // chassis.drive_to_pose(24,40,90);
  // IntakeBottom.spin(reverse);
  // IntakeTop.spin(reverse);
  // wait(2500,msec);
  // IntakeBottom.stop();
  // IntakeTop.stop();
  // chassis.drive_to_pose(24,24,90);
  // chassis.turn_to_angle(45);
  // chassis.drive_to_pose(46,2,45);
  // chassis.right_swing_to_angle(0);
  // Scraper1.set(true);
  // Scraper2.set(true);
  // chassis.drive_to_pose(70,2,0);
  // Scraper1.set(false);
  // Scraper2.set(false);

  

  //chassis.drive_to_pose(120,24,0);
  //display_screen();
  //wait(200000,msec);
  //chassis.turn_to_angle(90);
  //Scraper1.set(true);
  //Scraper2.set(true);
  //chassis.drive_to_pose(120,9,90);
  //IntakeBottom.spin(reverse);
  //wait(2500,msec);
  //IntakeBottom.stop();
  //chassis.drive_to_pose(120,24,90);
}
