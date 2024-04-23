#include "main.h"
#include <memory>

mpLib::Chassis chassis({11, -12, 13}, {-18, 19, -20}, 10, 11, 3.25, 450, 0.09);

pros::MotorGroup intakeMotors({14, -17});

pros::adi::Pneumatics leftWing('A', false);
pros::adi::Pneumatics rightWing('B', false);

mpLib::PID headingPID(.75, 0, 1);
mpLib::PID drivePID(15, 0, 0);

mpLib::Constraints constraints(76, 200, 1, 200, 0, 3.25);

void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Hello PROS User!");

  chassis.initialize();
  chassis.initControllers(&headingPID, &drivePID, 200, 200);
}

void disabled() {}

void competition_initialize() {}

void farSideRush() {
  chassis.setConstraints(450, 300, 0.09);
  // Grab lower mid ball
  chassis.followPath(new mpLib::Spline(
      {new mpLib::CubicBezier({32.82, -56.362}, {33.967, -38.71},
                              {31.903, -27.935}, {26.401, -9.823})}));
  intakeMotors.move(127);
  chassis.waitUntilSettled();
  pros::delay(100);
  // return to start
  chassis.followPath(new mpLib::Spline({new mpLib::CubicBezier(
                         {25.943, -2.559}, {36.947, -6.456}, {32.82, -35.342},
                         {31.903, -61.936})}),
                     true);
  chassis.waitUntilSettled();
  intakeMotors.move(-127);
  chassis.turnToHeading(55, 700);
  pros::delay(75);
  chassis.turnToHeading(-92, 550);
  // grab under hang bar
  chassis.followPath(new mpLib::Spline(
      {new mpLib::CubicBezier({33.508, -59.572}, {18.148, -59.457},
                              {22.504, -60.145}, {7.144, -60.03})}));
  intakeMotors.move(127);
  chassis.setConstraints(450, 300, 0.2);
  // back push
  chassis.followPath(new mpLib::Spline({new mpLib::CubicBezier(
                         {6.456, -62.624}, {45.2, -63.999}, {63.999, -57.809},
                         {64.916, -17.002})}),
                     true);
  pros::delay(800);
  leftWing.extend();
  pros::delay(600);
  leftWing.retract();
  chassis.waitUntilSettled();
  chassis.setConstraints(450, 500, 0.09);
  // back out of goal
  chassis.followPath(new mpLib::Spline(
      {new mpLib::CubicBezier({6.456, -62.624}, {45.2, -63.999},
                              {69.502, -61.248}, {64.916, -17.002})}));
  chassis.waitUntilSettled();
  chassis.turnToHeading(90, 300);
  chassis.turnToHeading(5, 600);
  // intakeMotors.move(127);
  // front push
  chassis.followPath(new mpLib::Spline(
      {new mpLib::CubicBezier({58.268, -44.67}, {58.268, -34.01},
                              {58.268, -19.409}, {58.268, -8.748})}));
  pros::delay(150);
  intakeMotors.move(-127);
  chassis.waitUntilSettled();
  chassis.setConstraints(450, 300, 0.15);
  pros::delay(150);
  // back out of goal
  chassis.followPath(new mpLib::Spline({new mpLib::CubicBezier(
                         {58.039, -24.954}, {57.58, -36.646}, {56.663, -42.979},
                         {53.454, -50.015})}),
                     true);
  chassis.waitUntilSettled();
  chassis.turnToHeading(-55, 700);
  intakeMotors.move(127);
  chassis.setConstraints(450, 300, 0.15);
  // grab left mid ball
  chassis.followPath(new mpLib::Spline(
      {new mpLib::CubicBezier({54.6, -50.86}, {42.22, -34.812},
                              {28.006, -31.216}, {11.041, -25.943})}));
  chassis.waitUntilSettled();
  chassis.turnToHeading(45, 600);
  // goal front push
  chassis.followPath(new mpLib::Spline(
      {new mpLib::CubicBezier({10.353, -23.192}, {20.899, -7.602},
                              {32.591, -8.978}, {50.473, -7.831})}));
  chassis.waitUntil(55);
  intakeMotors.move(-127);
  chassis.waitUntilSettled();
  pros::delay(200);
  chassis.turnToHeading(-90, 750);
  chassis.followPath(new mpLib::Spline(
      {new mpLib::CubicBezier({42.678, -4.163}, {28.465, -4.049},
                              {25.255, -5.381}, {13.104, -2.329})}));
  intakeMotors.move(127);
  chassis.waitUntilSettled();
  chassis.turnToHeading(90, 700);
  chassis.followPath(new mpLib::Spline({new mpLib::CubicBezier(
      {4.851, 0.264}, {29.267, 0.379}, {29.267, 0.379}, {53.683, 0.493})}));
  intakeMotors.move(-127);
  chassis.followPath(new mpLib::Spline({new mpLib::CubicBezier(
                         {44.283, 0.035}, {24.567, -0.194}, {24.567, -0.194},
                         {4.851, -0.424})}),
                     true);
}
void closeSideRush() {
  chassis.setConstraints(450, 200, 0.09);
  chassis.followPath(new mpLib::Spline(
      {new mpLib::CubicBezier({-33.435, -52.766}, {-34.581, -31.903},
                              {-30.684, -20.899}, {-26.328, -9.665})}));
  intakeMotors.move(127);
  chassis.waitUntilSettled();
  pros::delay(100);
  chassis.followPath(
      new mpLib::Spline(
          {new mpLib::CubicBezier({-28.391, -8.978}, {-40.313, -17.46},
                                  {-19.794, -26.172}, {-45.242, -34.884}),
           new mpLib::CubicBezier({-45.242, -34.884}, {-63.926, -39.698},
                                  {-61.404, -50.473}, {-49.024, -58.497})}),
      true);
  chassis.waitUntil(50);
  leftWing.extend();
  chassis.waitUntilSettled();
  chassis.turnToHeading(-110, 700);
  leftWing.retract();
  chassis.turnToHeading(95, 700);
  intakeMotors.move(-127);
  pros::delay(700);
  chassis.turnToHeading(-80, 1000);
  chassis.followPath(new mpLib::Spline({new mpLib::CubicBezier(
                         {-43.637, -56.663}, {-34.237, -59.414},
                         {-28.277, -59.185}, {-4.205, -58.726})}),
                     true);
  pros::delay(700);
  rightWing.extend();
  chassis.waitUntilSettled();
  chassis.turnToHeading(-90, 1000);
}

void closeSideSafe() {
  leftWing.extend();
  chassis.turnToHeading(-90, 600);
  chassis.turnToHeading(-70, 750);
  leftWing.retract();
  chassis.followPath(new mpLib::Spline({new mpLib::CubicBezier(
                         {-45.127, -54.141}, {-27.245, -62.853},
                         {-23.577, -57.809}, {-4.778, -53.224})}),
                     true);
  pros::delay(300);
  rightWing.extend();
  chassis.waitUntilSettled();
  chassis.turnToHeading(-85, 1000);
}
void autonomous() { farSideRush(); }

bool isForward = true;
void opcontrol() {
  // autonomous();
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  while (true) {
    if (master.get_digital(DIGITAL_L1)) {
      intakeMotors.move(127);
    } else if (master.get_digital(DIGITAL_L2)) {
      intakeMotors.move(-127);
    } else {
      intakeMotors.move(0);
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      leftWing.toggle();
      rightWing.toggle();
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      isForward = !isForward;
    }
    pros::lcd::print(0, "%d %d %d",
                     (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
                     (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
                     (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >>
                         0); // Prints status of the emulated screen LCDs

    // Arcade control scheme
    int dir =
        master.get_analog(ANALOG_LEFT_Y) *
        (isForward ? 1 : -1); // Gets amount forward/backward from left joystick
    int turn = master.get_analog(ANALOG_LEFT_X) *
               0.75; // Gets the turn left/right from right joystick
    chassis.tank(dir + turn, dir - turn);
    // leftMotors.move_vo
    pros::delay(20); // Run for 20 ms then update
  }
}
