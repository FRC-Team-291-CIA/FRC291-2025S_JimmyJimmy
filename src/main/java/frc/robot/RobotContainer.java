// Original RobotContainer.java File From BroncBotz3481/YAGSL-Example Release 2025.7.2

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Constants.ControllerDriverConstants;
import frc.robot.Constants.ControllerOperatorConstants;
import frc.robot.Constants.YAGSLConstants;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {
  final CommandJoystick m_controllerDriver = new CommandJoystick(ControllerDriverConstants.JOYSTICK_PORT);
  final CommandJoystick m_controllerOperator = new CommandJoystick(ControllerOperatorConstants.JOYSTICK_PORT);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> m_controllerDriver.getRawAxis(ControllerDriverConstants.AXIS_LEFT_Y) * -1,
      () -> m_controllerDriver.getRawAxis(ControllerDriverConstants.AXIS_LEFT_X) * -1)
      .withControllerRotationAxis(() -> m_controllerDriver.getRawAxis(ControllerDriverConstants.AXIS_RIGHT_X))
      .deadband(YAGSLConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(
      () -> m_controllerDriver.getRawAxis(
          ControllerDriverConstants.AXIS_RIGHT_X),
      () -> m_controllerDriver.getRawAxis(ControllerDriverConstants.AXIS_RIGHT_X))
      .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> -m_controllerDriver.getRawAxis(
          ControllerDriverConstants.AXIS_LEFT_Y),
      () -> -m_controllerDriver.getRawAxis(
          ControllerDriverConstants.AXIS_LEFT_X))
      .withControllerRotationAxis(() -> m_controllerDriver.getRawAxis(
          ControllerDriverConstants.AXIS_RIGHT_X))
      .deadband(
          YAGSLConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
      .withControllerHeadingAxis(() -> Math.sin(
          m_controllerDriver.getRawAxis(
              ControllerDriverConstants.AXIS_RIGHT_X) *
              Math.PI)
          *
          (Math.PI *
              2),
          () -> Math.cos(
              m_controllerDriver.getRawAxis(
                  ControllerDriverConstants.AXIS_RIGHT_X) *
                  Math.PI)
              *
              (Math.PI *
                  2))
      .headingWhile(true)
      .translationHeadingOffset(true)
      .translationHeadingOffset(Rotation2d.fromDegrees(
          0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {
    @SuppressWarnings("unused")
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    @SuppressWarnings("unused")
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    @SuppressWarnings("unused")
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    @SuppressWarnings("unused")
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    @SuppressWarnings("unused")
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation()) {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
          Rotation2d.fromDegrees(90));
      // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
          new ProfiledPIDController(5,
              0,
              0,
              new Constraints(5, 2)),
          new ProfiledPIDController(5,
              0,
              0,
              new Constraints(Units.degreesToRadians(360),
                  Units.degreesToRadians(180))));

      m_controllerDriver.button(ControllerDriverConstants.BUTTON_START)
          .onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      m_controllerDriver.button(ControllerDriverConstants.BUTTON_B).whileTrue(drivebase.sysIdDriveMotorCommand());
      m_controllerDriver.button(ControllerDriverConstants.BUTTON_X)
          .whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
              () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

      // driverXbox.b().whileTrue(
      // drivebase.driveToPose(
      // new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
      // );

    }
    if (DriverStation.isTest()) {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      m_controllerDriver.button(ControllerDriverConstants.BUTTON_X)
          .whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      m_controllerDriver.button(ControllerDriverConstants.BUTTON_Y)
          .whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      m_controllerDriver.button(ControllerDriverConstants.BUTTON_START).onTrue((Commands.runOnce(drivebase::zeroGyro)));
      m_controllerDriver.button(ControllerDriverConstants.BUTTON_BACK).whileTrue(drivebase.centerModulesCommand());
      m_controllerDriver.button(ControllerDriverConstants.BUTTON_BUMPER_TOP_LEFT).onTrue(Commands.none());
      m_controllerDriver.button(ControllerDriverConstants.BUTTON_BUMPER_TOP_RIGHT).onTrue(Commands.none());
    } else {
      m_controllerDriver.button(ControllerDriverConstants.BUTTON_A).onTrue((Commands.runOnce(drivebase::zeroGyro)));
      m_controllerDriver.button(ControllerDriverConstants.BUTTON_X)
          .onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      m_controllerDriver.button(ControllerDriverConstants.BUTTON_START).whileTrue(Commands.none());
      m_controllerDriver.button(ControllerDriverConstants.BUTTON_BACK)
          .whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      m_controllerDriver.button(ControllerDriverConstants.BUTTON_BUMPER_TOP_RIGHT).onTrue(Commands.none());
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
