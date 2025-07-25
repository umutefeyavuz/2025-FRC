// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.elevator.elevator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
     private Command m_autonomousCommand;
     private RobotContainer m_robotContainer;

     //additional codes for intake and elevator system.
     private Joystick joystick;
     private double startTime;
     //------------------------------------------------------------
     private PWMSparkMax intakemotor;
     private boolean togglePressed;
     private boolean getTrigger;
     private boolean intakeRunning;
     //------------------------------------------------------------
     private final Joystick stick = new Joystick(0);



    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        intakemotor = new PWMSparkMax(2); //spark max's channel for intake motor
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     **/
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        // As stated
    }

    @Override
    public void disabledPeriodic() {
        // As stated
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
        // As stated
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void teleopPeriodic() {
     // Joystick değerlerini alın (örneğin, stick nesnesi ile)
    double rtValue = stick.getRawAxis(3);
    double ltValue = stick.getRawAxis(2);

    // Elevator subsystem'inizden bir örnek oluşturduysanız (örneğin robotContainer veya Robot sınıfında)
    // Elevator kontrol metodunu çağırın:
    m_robotContainer.getelevator().runElevator(rtValue, ltValue);
}

    @Override
    public void simulationPeriodic() {

    }

    @Override
    public void simulationInit() {

    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();

        m_robotContainer.getTestingCommand().schedule();
    }

    @Override
    public void testPeriodic() {
    }
}