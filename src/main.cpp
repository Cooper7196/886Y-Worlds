#include "main.h"
#include <memory>

mpLib::Chassis chassis({11, -12, 13}, {-18, 19, -20}, 15, 11, 3.25, 450, 0.15);

pros::MotorGroup intakeMotors({14, -17});

pros::adi::Pneumatics wings('A', false);

mpLib::PID headingPID(5, 0, 2);
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

  chassis.followPath(
      new mpLib::CubicBezier({0, 0}, {0, 36}, {0, 36}, {-36, 36}), 450, 200,
      100);
  chassis.followPath(new mpLib::Spline(
      {new mpLib::CubicBezier({0, 0}, {0, 36}, {0, 36}, {-36, 36}),
       new mpLib::CubicBezier({0, 0}, {0, 36}, {0, 36}, {-36, 36})}));

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
