#include "main.h"
#include "gif-pros/gifclass.hpp"

using namespace pros;

/*
THINGS TO ADJUST:
Motor ports
High Goal Position
Distance Sensor Detecting Max Stack Size in Shoot()
Tracking Distances in Robot
Tracking Sensor Ports in Robot
Adjust kP,kI,kD for PIDMove() and PIDTurn()
Adjust Distance To Detected Disc variable in Shoot()
Roller Speed in opcontrol()
Download GIF onto SD card and plug into brain
Low/High speed and distance in Shoot()
*/

const float pi = 3.14159265359;
const double RollerSpeedMulti = 0.8; // 200 * 0.8 = 160

Motor left_front(1, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_COUNTS);
Motor left_back(2, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_COUNTS);
Motor_Group left({left_front,left_back});

Motor right_front(9, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_COUNTS);
Motor right_back(10, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_COUNTS);
Motor_Group right({right_front,right_back});

Motor flywheel1(4, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_COUNTS);
Motor flywheel2(5, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_COUNTS);
Motor_Group flywheel({flywheel1, flywheel2});

// Intake ratcheted to indexer
Motor intake1(6, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_COUNTS);
Motor intake2(7, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_COUNTS);
Motor_Group intake({intake1, intake2});

Distance distance1(11);
const int DistanceSensorDistanceFromTop = 20;

const float HighGoalPosX = 24.0;
const float HighGoalPosY = 120.0;

class Robot {
public:
  float x;
  float y;
  float heading;

  const float TrackingDistanceHorizontal = 1.0;
  const float TrackingDistanceBack = 1.0;
  const float TrackingWheelRadius = (2.75) / 2;

  const int EncoderTicksPerRevolution = 4096;

  const float InchesPerTick = 2.0 * pi * TrackingWheelRadius / EncoderTicksPerRevolution;

  ADIEncoder LeftTracking = {1, 2};
  ADIEncoder RightTracking = {3, 4};
  ADIEncoder BackTracking = {5, 6};

  Robot(float startingX, float startingY, float startingHeading) {
    x = startingX;
    y = startingY;
    heading = startingHeading;
  }
};

void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    lcd::set_text(2, "I was pressed!");
  } else {
    lcd::clear_line(2);
  }
}

void initialize() {
  lcd::initialize();
  lcd::set_text(1, "Hello PROS User!");

  lcd::register_btn1_cb(on_center_button);
}

void disabled() {}

Robot robot = {36, 12, 0.0}; // Left. Robot Right is {132,108,270};

void competition_initialize() {}

void OdomTick(void *) {
  lcd::clear();

  int CurrentRightPosition = 0;
  int CurrentLeftPosition = 0;
  int CurrentBackPosition = 0;

  int PrevRightPosition = 0;
  int PrevLeftPosition = 0;
  int PrevBackPosition = 0;
  while (true) {
    // Calculate Position and Orientation
    PrevRightPosition = CurrentRightPosition;
    PrevLeftPosition = CurrentLeftPosition;
    PrevBackPosition = CurrentBackPosition;

    CurrentRightPosition = robot.RightTracking.get_value();
    CurrentLeftPosition = robot.LeftTracking.get_value();
    CurrentBackPosition = robot.BackTracking.get_value();

    int DeltaN1 = CurrentLeftPosition - PrevLeftPosition;
    int DeltaN2 = CurrentRightPosition - PrevRightPosition;
    int DeltaN3 = CurrentBackPosition - PrevBackPosition;

    float DeltaTheta = robot.InchesPerTick * (DeltaN2 - DeltaN1) / robot.TrackingDistanceHorizontal;
    float DeltaX = robot.InchesPerTick * (DeltaN1 + DeltaN2) / 2.0;
    float DeltaY = robot.InchesPerTick * (DeltaN3 - (DeltaN2 - DeltaN1) * robot.TrackingDistanceBack / robot.TrackingDistanceHorizontal);

    float Theta = robot.heading + (DeltaTheta / 2.0);
    robot.x += DeltaX * cos(Theta) - DeltaY * sin(Theta);
    robot.y += DeltaX * sin(Theta) + DeltaY * cos(Theta);
    robot.heading += DeltaTheta;

    lcd::print(1, "Robot's X Position: %f", robot.x);
    lcd::print(2, "Robot's Y Position: %f", robot.y);
    lcd::print(3, "Robot's Heading: %f", robot.heading);

    pros::delay(10);
  }
}

void PIDMove(float distance) {
  float ValueLeft = robot.LeftTracking.get_value();
  float ValueRight = robot.RightTracking.get_value();

  float AverageValueEncoder;

  const double kP = 0.0;
  const double kI = 0.0;
  const double kD = 0.0;

  float integral;
  float derivative;

  float error;
  float prevError;
  float speed;

  while (fabs(error) > 0.1) {
    AverageValueEncoder = (((float)robot.LeftTracking.get_value() - ValueLeft) + ((float)robot.RightTracking.get_value()) - ValueRight) / 2;

    error = distance - AverageValueEncoder;
    integral += error;

    if (error == 0) {
      integral = 0;
    }
    if (fabs(error) > 40) {
      integral = 0;
    }

    derivative = error - prevError;

    prevError = error;
    speed = (kP * error) + (kI * integral) + (kD * derivative);

    left.move_voltage(speed);
    right.move_voltage(speed);
  }
}
void PIDTurn(float angle) {
  float StartingHeading = robot.heading;

  const double kP = 0.0;
  const double kI = 0.0;
  const double kD = 0.0;

  float integral;
  float derivative;

  float error;
  float prevError;
  float speed;

  while (fabs(error) > 0.1) {
    error = angle - (robot.heading - StartingHeading);
    integral += error;

    if (error == 0) {
      integral = 0;
    }
    if (fabs(error) > 40) {
      integral = 0;
    }

    derivative = error - prevError;

    prevError = error;
    speed = (kP * error) + (kI * integral) + (kD * derivative);

    left.move_voltage(speed);
    right.move_voltage(-speed);
  }
}

void TurnToHighGoal() {

  // Calculate needed heading to high goal
  float DistanceToHighGoalX = HighGoalPosX - robot.x;
  float DistanceToHighGoalY = HighGoalPosY - robot.y;

  float DegreesToHighGoal = 180 + atan(fabs(DistanceToHighGoalX / DistanceToHighGoalY));

  PIDTurn(DegreesToHighGoal - robot.heading);
}

void LowVelShotSpinUp() {
  const int LowSpeed = 80;
  const int LowSpeedVel = LowSpeed * (100 / 127);

  flywheel.move(LowSpeed);
  while (!(flywheel1.get_actual_velocity() >= LowSpeedVel || flywheel2.get_actual_velocity() >= LowSpeedVel)) {
    pros::delay(10);
  }
}

void HighVelShotSpinUp() {
  const int HighSpeed = 127;
  const int HighSpeedVel = HighSpeed * (100 / 127);

  flywheel.move(HighSpeed);
  while (!(flywheel1.get_actual_velocity() >= HighSpeedVel || flywheel2.get_actual_velocity() >= HighSpeedVel)) {
    pros::delay(10);
  }
}

void Shoot() {
  const int DistanceToDetectedDisc = 20;
  if (distance1.get() > DistanceToDetectedDisc) { // If no discs detected, exit function
    return;
  }
  const int DegreesTillShot = 1080; // Due to the gear ratio, the motors will have to spin 3 times for 1 full rotation of the linear slide drive gear

  float DistanceToHighGoalX = HighGoalPosX - robot.x;
  float DistanceToHighGoalY = HighGoalPosY - robot.y;
  float DistanceToHighGoal = sqrt(pow(DistanceToHighGoalX, 2) + pow(DistanceToHighGoalY, 2));

  if (DistanceToHighGoal <= 48) { // 48in (4 ft)
    LowVelShotSpinUp();
  } else {
    HighVelShotSpinUp();
  }

  intake.tare_position();
  while (distance1.get() <= DistanceToDetectedDisc) { // While discs are detected
    intake.move(-127);

    while (!((int)intake1.get_position() >= DegreesTillShot || (int)intake2.get_position() >= DegreesTillShot)) { // Wait until the motor has moved enough for a shot
      pros::delay(10);
    }

    intake.move(0);

    if (DistanceToHighGoal <= 48) { // 48in (4 ft)
      LowVelShotSpinUp();
    } else {
      HighVelShotSpinUp();
    }

    intake.tare_position();
  }
  flywheel.move(40);

  intake.move(127);
}

void autonomous() {
  Task OdomTask = {OdomTick};
  Gif gif("/usd/mygif.gif", lv_scr_act());
  // Something Something Auton
  gif.clean();
}

void opcontrol() {

  Gif gif("/usd/mygif.gif", lv_scr_act());

  Task OdomTask = {OdomTick};

  Controller master(E_CONTROLLER_MASTER);

  intake.move(127);

  flywheel.move(40);

  while (true) {
    left.move(master.get_analog(ANALOG_LEFT_Y) + master.get_analog(ANALOG_RIGHT_X));
    right.move(master.get_analog(ANALOG_LEFT_Y) - master.get_analog(ANALOG_RIGHT_X));

    if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_R1)) {
      TurnToHighGoal();
      master.rumble(".");
    }
    if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_R2)) {
      Shoot();
    }

    if (master.get_digital(E_CONTROLLER_DIGITAL_L2)) {
      intake.move(127 * RollerSpeedMulti);
    } else if (master.get_digital(E_CONTROLLER_DIGITAL_L1)) {
      intake.move(127 * -RollerSpeedMulti);
    } else {
      intake.move(127);
    }
    pros::delay(20);
  }
  gif.clean();
}