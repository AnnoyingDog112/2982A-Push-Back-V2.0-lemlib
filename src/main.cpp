#include "main.h"

MotorGroup right_motors({13, 14, 15}, MotorGearset::blue);
MotorGroup left_motors({-16, -17,-18}, MotorGearset::blue);

// drivetrain settings
Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              11.5, // 11.5 inch track width
                              Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2 (for now)
);

Imu imu(11);

Rotation horizontal_rotation_sensor(-8); 

TrackingWheel horizontal_tracking_wheel(
        &horizontal_rotation_sensor,
        Omniwheel::NEW_325, 
        1.5, // located on the tracking center
        450
);

OdomSensors sensors(nullptr, 
                            nullptr, 
                            nullptr,
                            nullptr,
                            &imu // inertial sensor(none right now)
);

// lateral PID controller
ControllerSettings lateral_controller(7, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              0, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              5, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
ControllerSettings angular_controller(1.7, // proportional gain (kP)
                                              0.2, // integral gain (kI)
                                              8, // derivative gain (kD)
                                              6, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              5, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


// // input curve for throttle input during driver control
// ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
//                                      10, // minimum output where drivetrain will move out of 127
//                                      1.019 // expo curve gain
// );

// // input curve for throttle input during driver control
ExpoDriveCurve throttle_curve(0, // joystick deadband out of 127
                                     0, // minimum output where drivetrain will move out of 127
                                     1.23 // expo curve gain
);

// // input curve for steer input during driver control
// ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
//                                   10, // minimum output where drivetrain will move out of 127
//                                   1.019 // expo curve gain
// );

ExpoDriveCurve steer_curve(0, // joystick deadband out of 127
                                  0, // minimum output where drivetrain will move out of 127
                                  1.23 // expo curve gain
);

// create the chassis
Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttle_curve,
                        &steer_curve
                        // &throttle_curve, 
                        // &steer_curve
);

Controller controller(E_CONTROLLER_MASTER);


Motor intake_stg_1_motor(19, MotorGearset::rpm_600);
Motor intake_stg_2_motor(9, MotorGearset::green);
Motor intake_stg_3_motor(20, MotorGearset::rpm_600);

adi::DigitalOut trapdoor('A', false);
adi::DigitalOut match_load('C', false);

int loop_delay_ms = 20;
bool intake_O_F = false;
bool trapdoor_O_F = false;
bool loader_O_F = false;


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	// lcd::initialize();
	// screen::print(1, "Hello PROS User!");
        chassis.calibrate(true);
        cout << "=== Program started ===" << endl;
        chassis.setPose(0, 0, 0);
	// Task screenTask([&]() {
        //         while (true) {
        //                 // print robot location to the brain screen
        //                 lcd::print(0, "X: %f", chassis.getPose().x); // x
        //                 lcd::print(1, "Y: %f", chassis.getPose().y); // y
        //                 lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
        //                 // log position telemetry
        //                 lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
        //                 // delay to save resources
        //                 delay(50);
        //         }
        // });
        // while (true) { // infinite loop
        //         // print measurements from the rotation sensor
        //         pros::lcd::print(1, "Rotation Sensor: %i", horizontal_rotation_sensor.get_position());
        //         pros::delay(10); // delay to save resources. DO NOT REMOVE
        // }
        chassis.setBrakeMode(motor_brake_mode_e::E_MOTOR_BRAKE_HOLD);
        // autonomous(); // Uncomment this line to run autonomous at the start of the program
        // lcd::register_btn1_cb(on_center_button);

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
        // start_angular_pid_logging_task(
        //     &chassis,
        //     &imu,
        //     angular_controller,
        //     90, // target in degrees
        //     5000, // timeout in ms
        //     loop_delay_ms
        // );
        start_lateral_pid_logging_task(
            &chassis,
            &imu,
            lateral_controller,
            48, // target in inches
            8000, // timeout in ms
            loop_delay_ms
        );
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
        chassis.setBrakeMode(motor_brake_mode_e::E_MOTOR_BRAKE_COAST);
        while (true) {
                // get left y and right y positions
                int leftY = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
                int leftX = controller.get_analog(E_CONTROLLER_ANALOG_LEFT_X);
                int rightY = controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y);
                int rightX = controller.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
                
                if (controller.get_digital(E_CONTROLLER_DIGITAL_X)){
                        if (!intake_O_F){
                                intake_stg_3_motor.move_velocity(180);
                                intake_O_F = true;
                        }
                        else{
                                intake_stg_3_motor.move_velocity(0);
                                intake_O_F = false;
                        }
                }
                else if (controller.get_digital(E_CONTROLLER_DIGITAL_B)){
                        if(!intake_O_F){
                                intake_stg_3_motor.move_velocity(-600);
                                intake_O_F = true;
                        }
                        else{
                                intake_stg_3_motor.move_velocity(0);
                                intake_O_F = false;
                        }
                }

                if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
                        intake_stg_2_motor.move_velocity(-600);
                }
                else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
                        intake_stg_2_motor.move_velocity(600);
                }
                else{
                        intake_stg_2_motor.move_velocity(0);
                }
                
                if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
                        intake_stg_1_motor.move_velocity(600);
                }
                else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
                        intake_stg_1_motor.move_velocity(-600);
                }
                else{
                        intake_stg_1_motor.move_velocity(0);
                }


                if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
                        if (!trapdoor_O_F){
                                trapdoor.set_value(true);
                                trapdoor_O_F = true;
                        }
                        else{
                                trapdoor.set_value(false);
                                trapdoor_O_F = false;
                        }
                }
                if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
                        if (!loader_O_F){
                                match_load.set_value(true);
                                loader_O_F = true;
                        }
                        else{
                                match_load.set_value(false);
                                loader_O_F = false;
                        }
                }
                
                
                // move the robot
                chassis.tank(leftY, rightY);

                delay(10);
        }
}
