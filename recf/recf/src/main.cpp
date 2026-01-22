#include "../include/main.h"
#include <cmath>
#include <sstream>
#include <string>
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
pros::Imu imu(13);
lemlib::OdomSensors sensors(
	nullptr,
	nullptr,
	nullptr,
	nullptr,
	&imu
);
lemlib::ControllerSettings lateral_controller(
	75,
	40,
	2,
	0.2,
	0,
	0,
	0,
	0,
	0
);
lemlib::ControllerSettings angular_controller(
	2,
	0,
	15,
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
lemlib::PID levelPID(4, 0, 16);
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Motor level(14, pros::v5::MotorGears::red);
pros::Rotation rotation(12);
pros::Motor intake(-7);
pros::adi::AnalogOut alignment(1);
pros::adi::DigitalIn lever(8);

int start_position;
int target[] = { 0, 5000, 8000 };
std::vector<char*> instr;

void initialize() {
	pros::lcd::initialize();
	chassis.calibrate();
	rotation.reset();
	start_position = rotation.get_position();

	FILE* file = fopen(lever.get_value() ? "/usd/autonL.rbs" : "/usd/autonR.rbs", "r");
	if (file != nullptr) {
		char buf[10000];
		fread(buf, 1, 10000, file);
		std::printf("%s\n", buf);
		char* add = strtok(buf, "\n");
		do {
			instr.push_back(add);
			add = strtok(NULL, "\n");
		} while (add);
	}
}

void autonomous() {
	intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	level.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	for (int pos = 0; pos <= instr.size(); pos++) {
		std::string i = instr[pos];
		std::vector<std::string> tok;
		std::stringstream ss(i);
		std::string word;
		while (ss >> word) { // The >> operator extracts space-separated words
			tok.push_back(word);
		}

		if (tok.size() == 0 || tok[0] == "//") {
			continue;
		}

		if (tok[0] == "MOVE") {
			std::vector<float> coords;
			std::stringstream ss(tok[1]);
			std::string coord;
			while (std::getline(ss, coord, ',')) { // The >> operator extracts space-separated words
				coords.push_back(std::stof(coord));
			}
			chassis.moveToPose(coords[0], coords[1], coords[2], std::stof(tok[2]));
		} else if (tok[0] == "INTAKE") {
			intake.move(tok[1] == "forward" ? 127 : tok[1] == "reverse" ? -127 : 0);
			if (tok[1] == "stop") {
				intake.brake();
			}
		} else if (tok[0] == "LEVEL") {
			bool close;
			do {
				int current = rotation.get_position() - start_position;
				int error;
				if (tok[1] == "up") {
					error = target[2] - current;
				} else if (tok[1] == "middle") {
					error = target[1] - current;
				} else if (tok[1] == "down") {
					error = target[0] - current;
				} else {
					throw 1;
				}
				level.move(levelPID.update(error));
				close = error <= 100;
			} while (!close);
		} else if (tok[0] == "ALIGN") {
			alignment.set_value(tok[1] == "out");
		}
	}
}

void opcontrol() {
	bool intake_moving = false;
	bool last_changed = false;
	bool aligned = false;
	int moveTo = -1;

	intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	level.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	while (true) {
		pros::lcd::print(0, "X: %f", chassis.getPose().x);
		pros::lcd::print(1, "Y: %f", chassis.getPose().y);
		pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
		pros::lcd::print(3, "Level: %d", rotation.get_position() - start_position);

		int left_thumbstick = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int right_thumbstick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		chassis.curvature(left_thumbstick, right_thumbstick, false);

		if (controller.get_digital(DIGITAL_B) == 1 && intake.get_power() > 0.1) {
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

		int current = rotation.get_position() - start_position;
		if (controller.get_digital(DIGITAL_L2) == 1) {
			level.move(-127);
		} else if (controller.get_digital(DIGITAL_R2) == 1) {
			level.move(127);
		} else if (controller.get_digital(DIGITAL_UP) == 1 || moveTo == 2) {
			level.move(levelPID.update(current - target[2]));
			if (abs(target[2] - current) <= 100) {
				moveTo = -1;
				level.brake();
			} else {
				moveTo = 2;
			}
		} else if (controller.get_digital(DIGITAL_LEFT) == 1 || moveTo == 1) {
			level.move(levelPID.update(current - target[1]));
			if (abs(target[1] - current) <= 100) {
				moveTo = -1;
				level.brake();
			} else {
				moveTo = 1;
			}
		} else if (controller.get_digital(DIGITAL_DOWN) == 1 || moveTo == 0) {
			level.move(levelPID.update(current - target[0]));
			if (abs(target[0] - current) <= 100) {
				moveTo = -1;
				level.brake();
			} else {
				moveTo = 0;
			}
		} else {
			level.move(0);
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
