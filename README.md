# Iron Tank Robot Code

This repository contains the code for the Iron Tank robot, developed using the WPILib framework and CTRE Phoenix libraries. The project is structured to support multiple subsystems and commands, following the Command-Based programming paradigm.

## Branches

### Main Branch
The `main` branch contains the core functionality of the robot, including:
- Drive subsystem for controlling the robot's movement.
- Commands for autonomous and teleoperated modes.
- Integration with Limelight for vision-based targeting.

### Test Branch
The `Test` branch includes additional functionality for testing purposes:
- A **Test Motor** subsystem (`TestSubsystem`) for experimenting with a TalonFX motor (CAN ID 2).
- Commands to control the test motor, such as setting its position and stopping it.
- Logging of motor data, including output, position, velocity, temperature, and current.

> **Note:** The test motor is only available in the `Test` branch. If you are working with the test motor, ensure you are on the `Test` branch.


## Subsystems

### DriveSubsystem
- Controls the robot's drivetrain using VictorSPX motor controllers.
- Supports arcade drive and tank drive modes.
- Logs motor output and temperature for debugging.

### TestSubsystem (Test Branch Only)
- Controls a TalonFX motor for testing purposes.
- Configurable PID gains for position control.
- Logs detailed motor data, including position, velocity, temperature, and current.

## Commands

### AimAtAprilTagCommand
- Uses a PID controller to align the robot with an AprilTag using Limelight.

### FollowAprilTagCommand
- Combines PID controllers for turning and distance to follow an AprilTag.

### TestMotorCommand (Test Branch Only)
- Controls the test motor in the `TestSubsystem`.
- Allows setting motor position dynamically.

## Usage

### Building the Project
Run the following command to build the project:
```sh
./gradlew build
```

### Deploying to the Robot
Use the following command to deploy the code to the robot:
```sh
./gradlew deploy
```

## Dependencies
- WPILib: For robot programming framework.
- CTRE Phoenix 6: For motor controllers and sensors.
- AdvantageKit: For logging and telemetry.

## Notes
- Ensure you are on the correct branch (`main` or `Test`) based on your requirements.
- The `Test` branch includes experimental features and is not intended for production use.

## License
This project is licensed under the WPILib BSD license. See the WPILib-License.md[WPILIB-License.md] file for details.
