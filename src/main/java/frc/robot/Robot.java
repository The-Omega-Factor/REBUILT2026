// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.Alliance;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    private final NetworkTable robot;
    private final DoublePublisher batteryPublisher;
    private final DoublePublisher timerPublisher;

    public Robot() {
        m_robotContainer = new RobotContainer();

        robot = NetworkTableInstance.getDefault().getTable("Robot");
        timerPublisher = robot.getDoubleTopic("Timer").publish();
        batteryPublisher = robot.getDoubleTopic("Voltage").publish();
        batteryPublisher.setDefault(-1.0);
        timerPublisher.setDefault(0.0);
    }

    @Override
    public void robotInit() {
        
    }

    @Override
    public void robotPeriodic() {

        PortForwarder.add(5800, "limelight.local", 5800); 
        PortForwarder.add(5801, "limelight.local", 5801); 

        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
        Alliance.update();

        batteryPublisher.set(RobotController.getBatteryVoltage());
        timerPublisher.set(DriverStation.getMatchTime());
    }
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {

    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    }
  

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
