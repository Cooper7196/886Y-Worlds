#include "main.h"
#include <memory>

mpLib::Chassis chassis({11, -12, 13}, {-18, 19, -20}, 15, 11, 3.25, 450, 0.09);

pros::MotorGroup intakeMotors({14, -17});

pros::adi::Pneumatics wings('A', false);

mpLib::PID headingPID(1, 0, 0.5);
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

void autonomous() {
  chassis.driveDistancePID(4);
  wings.extend();
  pros::delay(1000);
  std::cout << chassis.getHeading() << std::endl;
  chassis.swingTo(-34, true, 3000);
  chassis.driveDistancePID(-19, 40, 6000);
}

bool isForward = true;
void opcontrol() {
  chassis.followPath(new mpLib::Spline(
      {new mpLib::CubicBezier({32.591, -55.288}, {32.82, -32.362},
                              {17.919, -25.026}, {6.685, -5.539})}));
  chassis.waitUntil(45);
  intakeMotors.move(120);
  chassis.waitUntilSettled();
  chassis.followPath(new mpLib::Spline({new mpLib::CubicBezier(
                         {5.997, -2.329}, {12.646, -16.085}, {32.362, -41.761},
                         {30.069, -62.395})}),
                     true);
  chassis.waitUntil(10);
  intakeMotors.move(50);
  chassis.waitUntilSettled();
  intakeMotors.move(-70);
  chassis.turnToHeading(55, 1500);
  pros::delay(100);
  chassis.turnToHeading(-95, 1500);
  chassis.followPath(new mpLib::Spline(
      {new mpLib::CubicBezier({39.24, -58.039}, {23.192, -58.039},
                              {23.192, -58.039}, {7.144, -58.039})}));
  intakeMotors.move(90);
  chassis.followPath(new mpLib::Spline({new mpLib::CubicBezier(
                         {2.559, -58.268}, {23.421, -57.809}, {64.916, -66.063},
                         {63.312, -26.63})}),
                     true);
  chassis.waitUntil(5);
  intakeMotors.move(40);
  chassis.waitUntilSettled();
  // chassis.followPath(new mpLib::Spline(
  //     {new mpLib::CubicBezier({0, 0}, {0, 8}, {0, 16}, {0, 24})}));
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  while (true) {
    if (master.get_digital(DIGITAL_L1)) {
      intakeMotors.move(90);
    } else if (master.get_digital(DIGITAL_L2)) {
      intakeMotors.move(-127);
    } else {
      intakeMotors.move(0);
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      wings.toggle();
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
