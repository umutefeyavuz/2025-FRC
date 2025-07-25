# Swerve Drive with Elevator System (FRC 2025)

This project contains the source code for a robot equipped with a **swerve drive system** and a motorized **elevator mechanism**, developed for the 2025 FRC season. It is written in Java using WPILib and follows a modular command-based architecture.

## Features

- Fully functional swerve drivetrain using gyroscope-based field-relative movement.
- Elevator subsystem controlled via PID logic with predefined target heights.
- Command-based structure allowing modular control and autonomous development.
- SmartDashboard integration for runtime monitoring and control.

## Project Structure

```
src/
└── main/
    └── java/
        └── frc/
            └── robot/
                ├── Robot.java                # Main robot file
                ├── Constants.java            # Constants used across subsystems
                ├── RobotContainer.java       # Binds subsystems and commands
                ├── subsystems/
                │   ├── SwerveDrivetrain.java # Drivetrain implementation
                │   └── Elevator.java         # Elevator control logic
                └── commands/
                    ├── DriveCommand.java     # Swerve driving command
                    └── MoveElevatorToHeight.java
```

## File Explanations

### Robot.java
The entry point of the program. Initializes all robot modes (autonomous, teleop, etc.) and calls periodic methods.

### Constants.java
Holds all robot-wide constants such as motor IDs, PID gains, and measurements for easy tuning and reuse.

### RobotContainer.java
Initializes and binds all subsystems and commands. Also maps joystick inputs to command triggers.

### subsystems/SwerveDrivetrain.java
Contains logic to control a swerve drivetrain using:
- Kinematics (swerve module translation & rotation)
- Field-relative movement using gyro
- Module speed control via motor output
- Encoder feedback for each module

### subsystems/Elevator.java
Controls a linear elevator mechanism, using:
- PID logic for position control
- Predefined height positions
- Sensor feedback (encoder or limit switch)

### commands/DriveCommand.java
Default command that handles real-time drive input from the driver. Reads joystick axes to set robot velocity and heading.

### commands/MoveElevatorToHeight.java
Parameterized command that moves the elevator to a specific setpoint. Used with buttons or autonomous logic.

## Requirements

- WPILib (2025 release)
- Java 17+
- FRC RoboRIO hardware
- NavX or other gyro sensor
- CAN-based swerve modules
- Motor controller for elevator (TalonFX, SparkMax, etc.)

## Getting Started

1. Clone the repository:
   ```
   git clone https://github.com/YOUR_USERNAME/YOUR_REPOSITORY_NAME.git
   ```
2. Open with VS Code and ensure the WPILib extension is installed.
3. Deploy to robot using:
   ```
   Ctrl+Shift+P > WPILib: Deploy Robot Code
   ```

## Controls

- **Joystick/Yoke** used for controlling swerve drive.
- **Preset buttons** for sending the elevator to predefined positions.

## Author

Created and maintained by **Umut Efe Yavuz** for learning, competition preparation, and contribution to the FRC community.

## License

This project is open source and provided for educational purposes. Feel free to use and adapt it under the [MIT License](https://opensource.org/licenses/MIT).
