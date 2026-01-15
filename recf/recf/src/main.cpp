#include "../include/main.h"
#include <cmath>
#include "lemlib/api.hpp"

pros::MotorGroup left_motors({ -1, 2, -3 }, pros::MotorGearset::green);
pros::MotorGroup right_motors({ 4, -5, 6 }, pros::MotorGearset::green);
lemlib::Drivetrain drivetrain(
	&left_motors,
	&right_motors,
	12.75,
	lemlib::Omniwheel::NEW_4,
	320,
	4
);
pros::Imu imu(7);
lemlib::OdomSensors sensors(
	nullptr,
	nullptr,
	nullptr,
	nullptr,
	&imu
);
lemlib::ControllerSettings lateral_controller(
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0
);
lemlib::ControllerSettings angular_controller(
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0
);
lemlib::Chassis chassis(
	drivetrain,
	lateral_controller,
	angular_controller,
	sensors
);
lemlib::PID levelPID(0, 0, 0);
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Motor level(8);
pros::Rotation rotation(9);
pros::Motor intake(-10);
pros::adi::AnalogOut alignment(1);

int target[] = { 0, 3000, 4500 };

void initialize() {
	pros::lcd::initialize();
	chassis.calibrate();
	rotation.reset_position();
}

void autonomous() {
	//TODO
}

void opcontrol() {
	bool intake_moving = false;
	bool last_changed = false;
	bool aligned = false;

	while (true) {
		pros::lcd::print(0, "X: %f", chassis.getPose().x);
		pros::lcd::print(1, "Y: %f", chassis.getPose().y);
		pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);

		int left_thumbstick = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int right_thumbstick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		chassis.curvature(left_thumbstick, right_thumbstick, false);

		if (controller.get_digital(DIGITAL_B) == 1 && intake.get_power() > 0.1) {
			intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			intake.brake();
			intake_moving = false;
		} else if (controller.get_digital(DIGITAL_A) == 1) {
			intake.move(127);
			intake_moving = true;
		} else if (controller.get_digital(DIGITAL_R1) == 1) {
			intake.move(127);
			intake_moving = false;
		} else if (controller.get_digital(DIGITAL_L1) == 1) {
			intake.move(-127);
			intake_moving = false;
		} else if (abs(intake.get_current_draw()) <= 5000 && !intake_moving) {
			intake.brake();
		}

		int current = rotation.get_position();
		if (controller.get_digital(DIGITAL_L2) == 1) {
			level.move(-127);
		} else if (controller.get_digital(DIGITAL_R2) == 1) {
			level.move(127);
		} else if (controller.get_digital(DIGITAL_UP) == 1) {
			level.move_relative(levelPID.update(target[2] - current), 100);
		} else if (controller.get_digital(DIGITAL_LEFT) == 1) {
			level.move_relative(levelPID.update(target[1] - current), 100);
		} else if (controller.get_digital(DIGITAL_DOWN) == 1) {
			level.move_relative(levelPID.update(target[0] - current), 100);
		} else {
			level.move(0);
			level.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			level.brake();
		}

		if (controller.get_digital(DIGITAL_X) == 1) {
			if (aligned && !last_changed) {
				alignment.set_value(false);
				aligned = false;
			} else if (!aligned && !last_changed) {
				alignment.set_value(true);
				aligned = true;
			}
			last_changed = true;
		} else {
			last_changed = false;
		}

		pros::delay(35);
	}
}
