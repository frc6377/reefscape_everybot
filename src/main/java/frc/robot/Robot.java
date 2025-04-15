// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CANdleSignalingSubsystem;
import frc.robot.subsystems.CANdleSignalingSubsystem.LightState;
import java.io.ByteArrayInputStream;
import java.io.UnsupportedEncodingException;
import java.util.NoSuchElementException;
import org.littletonrobotics.junction.*;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.*;

public class Robot extends LoggedRobot {

  public static final boolean isCompetition = false;

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  @SuppressWarnings("resource") // Ignore not closing PowerDistribution instance
  public Robot() {
    Logger.recordMetadata("ProjectName", "Reefscape_2025"); // Set a metadata value

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else {
      try {
        /*
          findReplayLog() prompts the user for a log file path to replay, requiring input from the
          user on the terminal.1
          This is a undesired interaction, so using setIn() to provide an empty line to the prompt.

          findReplayLog() looks for the log file in AdvantageKit first, so this doesn't interrupt
          our typical use case.
        */
        System.setIn(new ByteArrayInputStream("\n".getBytes("UTF-8")));
        String logPath = LogFileUtil.findReplayLog();
        if (logPath != null) {
          setUseTiming(false); // Run as fast as possible

          Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
          Logger.addDataReceiver(
              new WPILOGWriter(
                  LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        }
      } catch (NoSuchElementException
          | StringIndexOutOfBoundsException
          | UnsupportedEncodingException ex) {
        System.out.println(); // put new line after prompt
        DriverStation.reportWarning("No log file found, simulating as normal. \n", false);
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      }
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
    // be added.
    m_robotContainer = new RobotContainer();

    CANdleSignalingSubsystem signaling = m_robotContainer.signalingSubsystem;

    CommandScheduler.getInstance()
        .onCommandInitialize(
            command -> {
              if (command.getName().equals("intakeCoralCommand")
                  || command.getName().equals("ejectCoralCommand")) {
                signaling.setState(LightState.CORAL_ACTION);
              } else if (command.getName().equals("intakeAlgaeCommand")
                  || command.getName().equals("ejectAlgaeCommand")) {
                signaling.setState(LightState.ALGAE_ACTION);
              }
            });

    CommandScheduler.getInstance()
        .onCommandFinish(
            command -> {
              if (!DriverStation.isDisabled()) {
                if (DriverStation.isAutonomous()) {
                  signaling.setState(LightState.AUTON_IDLE);
                  return;
                } else if (RobotContainer.coralMode) {
                  signaling.setState(LightState.CORAL_MODE);
                } else if (!RobotContainer.coralMode) {
                  signaling.setState(LightState.ALGAE_MODE);
                }
              }
            });

    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command -> {
              if (!DriverStation.isDisabled()) {
                if (DriverStation.isAutonomous()) {
                  signaling.setState(LightState.AUTON_IDLE);
                  return;
                } else if (RobotContainer.coralMode) {
                  signaling.setState(LightState.CORAL_MODE);
                } else if (!RobotContainer.coralMode) {
                  signaling.setState(LightState.ALGAE_MODE);
                }
              }
            });
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    CANdleSignalingSubsystem signaling = m_robotContainer.signalingSubsystem;
    signaling.setState(LightState.DISABLED);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    CANdleSignalingSubsystem signaling = m_robotContainer.signalingSubsystem;

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      signaling.setState(LightState.AUTON_IDLE);
    }
  }

  @Override
  public void autonomousPeriodic() {
    if (m_autonomousCommand != null && m_autonomousCommand.isFinished()) {
      CANdleSignalingSubsystem signaling = m_robotContainer.signalingSubsystem;
      signaling.setState(LightState.AUTON_FINISH);
    }
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.algaeScorerSubsystem.setIntakeAngleCommand(
        Constants.AlgaeScorerConstants.PIVOT_STOW_ANGLE);

    CANdleSignalingSubsystem signaling = m_robotContainer.signalingSubsystem;
    if (RobotContainer.coralMode) {
      signaling.setState(LightState.CORAL_MODE);
    } else if (!RobotContainer.coralMode) {
      signaling.setState(LightState.ALGAE_MODE);
    }
  }

  @Override
  public void teleopPeriodic() {}

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

  public String[] getScheduledCommands() {
    NetworkTable table =
        NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("Scheduler");
    if (table != null) {
      String[] scheduledCommands = table.getEntry("Names").getStringArray(new String[0]);
      return scheduledCommands;
    }
    return null;
  }
}
