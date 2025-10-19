#include "main.h"

Controller controller(E_CONTROLLER_MASTER);

MotorGroup left_motors({13, 14, 15}, MotorGearset::blue);
MotorGroup right_motors({-16, -17,-18}, MotorGearset::blue);

// drivetrain settings
Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              11.5, // 11.5 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2 (for now)
);

pros::Imu imu(11);

pros::Rotation horizontal_rotation_sensor(12); 

TrackingWheel left_vertical_tracking_wheel(
	&left_motors,
	lemlib::Omniwheel::NEW_325, 
	-5.75, // located on the tracking center
	450
);

TrackingWheel right_vertical_tracking_wheel(
	&right_motors,
	lemlib::Omniwheel::NEW_325, 
	5.75, // located on the tracking center
	450
);

TrackingWheel horizontal_tracking_wheel(
        &horizontal_rotation_sensor,
        lemlib::Omniwheel::NEW_325, 
        0, // located on the tracking center
        450
);

OdomSensors sensors(&left_vertical_tracking_wheel, // left drivtrain tracking wheel using drivetrain IMEs
                            &right_vertical_tracking_wheel, // right drivtrain tracking wheel using drivetrain IMEs
                            &horizontal_tracking_wheel, // Set to none(for now as we don't have one)
                            nullptr, // Set to none(for now as we don't have one)
                            &imu // inertial sensor(none right now)
);

// lateral PID controller
ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


// // input curve for throttle input during driver control
// lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
//                                      10, // minimum output where drivetrain will move out of 127
//                                      1.019 // expo curve gain
// );

// // input curve for steer input during driver control
// lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
//                                   10, // minimum output where drivetrain will move out of 127
//                                   1.019 // expo curve gain
// );

// create the chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        nullptr,
                        nullptr
                        // &throttle_curve, 
                        // &steer_curve
);

int loop_delay_ms = 25;

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	delay(500); // allow time for LCD to initialize
	lcd::initialize();
	// screen::print(1, "Hello PROS User!");
        delay(1000);
        chassis.calibrate();
        std::cout << "=== Program started ===" << std::endl;
        // screen::
        // chassis.setPose(0, 0, 0);
	
        // pros::lcd::register_btn1_cb(on_center_button);

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() 
{
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
        while (true) {
                int start_t = pros::millis();
                // get left y and right y positions
                int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
                int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

                // move the robot
                chassis.tank(leftY, rightY);

                if (millis() - start_t <= loop_delay_ms) delay(loop_delay_ms - (millis() - start_t));
        }
}
