// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlgaeScorerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.CANAlgaeManipulatorSubsystem;
import frc.robot.subsystems.CANCoralScorerSubsystem;
import frc.robot.subsystems.CANDriveSubsystem;
import java.util.HashMap;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  private final CANCoralScorerSubsystem coralScorerSubsystem = new CANCoralScorerSubsystem();
  private final CANAlgaeManipulatorSubsystem algaeScorerSubsystem =
      new CANAlgaeManipulatorSubsystem();

  public enum RobotMode{
    ALGAE,
    CORAL
  }
  public static RobotMode currentRobotMode = RobotMode.CORAL;

  // The driver's controller
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  // The operator's controller
  private final CommandXboxController operatorController =
      new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  // Registering Auto Commands
  // The autonomous chooser
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    registerAutoCommands();
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    configureBindings();
  }

  public static RobotMode getRobotMode(){
    return currentRobotMode;
  }

  private Subsystem getRequiredSubsystem(){
    if(currentRobotMode == RobotMode.CORAL){
      return algaeScorerSubsystem;
    } else {
      return coralScorerSubsystem;
    }
  }

  private static void toggleRobotMode(){
    if(currentRobotMode == RobotMode.CORAL){
      currentRobotMode = RobotMode.ALGAE;
    } else {
      currentRobotMode = RobotMode.CORAL;
    }
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  public void registerAutoCommands() {
    HashMap<String, Command> autonCommands = new HashMap<String, Command>();

    autonCommands.put("ScoreCoralL1", coralScorerSubsystem.ejectCommand());

    NamedCommands.registerCommands(autonCommands);
  }

  public void configureBindings() {
    if (currentRobotMode == RobotMode.CORAL){
      driveSubsystem.setDefaultCommand(
        driveSubsystem.arcadeDrive(
            () -> cubicCurve(() -> -driverController.getLeftY(), DriveConstants.CONTROL_CURVE_INTENSITY), () -> -driverController.getRightX()));

      driverController.leftTrigger().whileTrue(coralScorerSubsystem.intakeCommand());
      driverController.rightTrigger().whileTrue(coralScorerSubsystem.ejectCommand());
      driverController.leftBumper().onTrue(coralScorerSubsystem.toggleIntakeSpeedCommand());
    } else if(currentRobotMode == RobotMode.ALGAE){
      driveSubsystem.setDefaultCommand(
        driveSubsystem.arcadeDrive(
            () -> cubicCurve(() -> driverController.getLeftY(), DriveConstants.CONTROL_CURVE_INTENSITY), () -> driverController.getRightX()));

      driverController.rightTrigger().whileTrue(algaeScorerSubsystem.intakeAlgaeCommand());
      driverController.leftTrigger().whileTrue(algaeScorerSubsystem.outakeAlgaeCommand());
      driverController.rightBumper().onTrue(algaeScorerSubsystem.togglePivotCommand());
    }

    driverController.a().onTrue(Commands.runOnce(() -> toggleRobotMode(), getRequiredSubsystem()));
  }

  public double cubicCurve(DoubleSupplier input, double intensity) {
    return intensity * Math.pow(input.getAsDouble(), 3) + (1 - intensity) * input.getAsDouble();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
