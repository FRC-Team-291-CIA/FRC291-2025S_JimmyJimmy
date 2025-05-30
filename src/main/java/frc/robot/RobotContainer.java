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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.NamedCommands;

import swervelib.SwerveInputStream;

import frc.robot.Constants.ControllerDriverConstants;
import frc.robot.Constants.ControllerOperatorConstants;
import frc.robot.Constants.FlapConstants;
import frc.robot.Constants.YAGSLConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem.SpeedState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.flap.FlapSubsystem;
import frc.robot.subsystems.flap.FlapSubsystem.FlapState;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.led.LedSubsystem.DeviceLEDState;
import frc.robot.subsystems.led.LedSubsystem.StripAnimationState;
import frc.robot.commands.AlgaeFlapCommand;
import frc.robot.commands.AutoScoreCoralCommand;
import frc.robot.commands.IntakeCoralCommand;

import frc.robot.Constants.CodeConstants;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;

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
        CommandJoystick m_controllerDeveloper;

        // The robot's subsystems and commands are defined here...
        private final SwerveSubsystem m_drivebase = new SwerveSubsystem(
                        new File(Filesystem.getDeployDirectory(), "swerve"));

        private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
        private final CoralSubsystem m_coral = new CoralSubsystem();
        private final FlapSubsystem m_flap = new FlapSubsystem();
        private final ClimberSubsystem m_climber = new ClimberSubsystem();
        private final LedSubsystem m_led = new LedSubsystem();

        /**
         * Converts driver input into a field-relative ChassisSpeeds that is controlled
         * by angular velocity.
         */
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_drivebase.getSwerveDrive(),
                        () -> m_controllerDriver.getRawAxis(ControllerDriverConstants.AXIS_LEFT_Y) * -1,
                        () -> m_controllerDriver.getRawAxis(ControllerDriverConstants.AXIS_LEFT_X) * -1)
                        .withControllerRotationAxis(
                                        () -> m_controllerDriver.getRawAxis(ControllerDriverConstants.AXIS_RIGHT_X)
                                                        * -1)
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

        SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(m_drivebase.getSwerveDrive(),
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

        SendableChooser<Command> m_chooser = new SendableChooser<>();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                DogLog.setOptions(new DogLogOptions().withCaptureNt(true));

                switch (CodeConstants.DEV_CONTROLLER_MODE) {
                        case ON:
                                m_controllerDeveloper = new CommandJoystick(2);
                                break;
                        case OFF:

                                break;

                }
                NamedCommands.registerCommand("Coral Intake", new IntakeCoralCommand(m_coral));
                NamedCommands.registerCommand("Score Level Four",
                                new AutoScoreCoralCommand(m_coral, m_elevator, ElevatorState.CORAL_LEVEL_FOUR));
                NamedCommands.registerCommand("Score Level Three",
                                new AutoScoreCoralCommand(m_coral, m_elevator, ElevatorState.CORAL_LEVEL_THREE));
                NamedCommands.registerCommand("Score Level Two",
                                new AutoScoreCoralCommand(m_coral, m_elevator, ElevatorState.CORAL_LEVEL_TWO));
                NamedCommands.registerCommand("Algae Flap Command",
                                new AlgaeFlapCommand(m_elevator, m_flap, m_drivebase, ElevatorState.CORAL_LEVEL_TWO));

                m_chooser.setDefaultOption("DO NOTHING", Commands.none());
                m_chooser.addOption("FORWARD ONLY", m_drivebase.getAutonomousCommand("FORWARD ONLY"));
                m_chooser.addOption("SCORE L3 CENTER REEF TO G",
                                m_drivebase.getAutonomousCommand("SCORE L3 CENTER REEF TO G"));
                m_chooser.addOption("SCORE L3 CENTER REEF TO H",
                                m_drivebase.getAutonomousCommand("SCORE L3 CENTER REEF TO H"));
                m_chooser.addOption("SCORE L4 CENTER REEF TO G",
                                m_drivebase.getAutonomousCommand("SCORE L4 CENTER REEF TO G"));
                m_chooser.addOption("SCORE L4 CENTER REEF TO H",
                                m_drivebase.getAutonomousCommand("SCORE L4 CENTER REEF TO H"));
                m_chooser.addOption("ALGAE CENTER REEF TO H",
                                m_drivebase.getAutonomousCommand("ALGAE CENTER REEF TO H"));
                m_chooser.addOption("ALGAE CENTER REEF TO G",
                                m_drivebase.getAutonomousCommand("ALGAE CENTER REEF TO G"));
                SmartDashboard.putData(m_chooser);

                // Configure the trigger bindings
                configureBindings();
                RobotModeTriggers.teleop().onTrue(Commands.parallel(
                                Commands.runOnce(() -> m_drivebase.setWantedSpeedState(SpeedState.NORMAL), m_drivebase),
                                Commands.runOnce(() -> DogLog.setEnabled(true))));

                RobotModeTriggers.autonomous().onTrue(Commands.parallel(
                                Commands.runOnce(() -> m_drivebase.setWantedSpeedState(SpeedState.NORMAL)),
                                Commands.runOnce(() -> DogLog.setEnabled(true))));

                DriverStation.silenceJoystickConnectionWarning(true);

                new Trigger(m_drivebase.onFastSelected())
                                .onTrue(Commands.runOnce(() -> m_led.setStripColor(0, 0, 255), m_led));

                new Trigger(m_drivebase.onNormalSelected())
                                .onTrue(Commands.runOnce(() -> m_led.setStripColor(0, 255, 0), m_led));

                new Trigger(m_drivebase.onSlowSelected())
                                .onTrue(Commands.runOnce(() -> m_led.setStripColor(255, 255, 0), m_led));

                new Trigger(m_drivebase.onVerySlowSelected())
                                .onTrue(Commands.runOnce(() -> m_led.setStripColor(255, 0, 0), m_led));

                SmartDashboard.putData("Flap Command", m_flap);
                SmartDashboard.putData("Coral Command", m_coral);
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
                Command driveFieldOrientedDirectAngle = m_drivebase.driveFieldOriented(driveDirectAngle);
                Command driveFieldOrientedAnglularVelocity = m_drivebase.driveFieldOriented(driveAngularVelocity);
                @SuppressWarnings("unused")
                Command driveRobotOrientedAngularVelocity = m_drivebase.driveFieldOriented(driveRobotOriented);
                @SuppressWarnings("unused")
                Command driveSetpointGen = m_drivebase.driveWithSetpointGeneratorFieldRelative(
                                driveDirectAngle);
                Command driveFieldOrientedDirectAngleKeyboard = m_drivebase
                                .driveFieldOriented(driveDirectAngleKeyboard);
                @SuppressWarnings("unused")
                Command driveFieldOrientedAnglularVelocityKeyboard = m_drivebase
                                .driveFieldOriented(driveAngularVelocityKeyboard);
                @SuppressWarnings("unused")
                Command driveSetpointGenKeyboard = m_drivebase.driveWithSetpointGeneratorFieldRelative(
                                driveDirectAngleKeyboard);

                if (RobotBase.isSimulation()) {
                        // m_drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
                        m_drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
                } else {
                        m_drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
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
                                        .onTrue(Commands.runOnce(() -> m_drivebase
                                                        .resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
                        m_controllerDriver.button(ControllerDriverConstants.BUTTON_B)
                                        .whileTrue(m_drivebase.sysIdDriveMotorCommand());
                        m_controllerDriver.button(ControllerDriverConstants.BUTTON_X)
                                        .whileTrue(Commands.runEnd(
                                                        () -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                        () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
                }

                // ===== Swerve =====
                // === Driver Controller ===
                m_controllerDriver.button(ControllerDriverConstants.BUTTON_A)
                                .onTrue(Commands.runOnce(m_drivebase::zeroGyro));

                m_controllerDriver.button(ControllerDriverConstants.BUTTON_BUMPER_TOP_LEFT)
                                .onTrue(Commands.runOnce(m_drivebase::speedShiftDown));

                m_controllerDriver.button(ControllerDriverConstants.BUTTON_BUMPER_TOP_RIGHT)
                                .onTrue(Commands.runOnce(m_drivebase::speedShiftUp));

                // ===== Elevator =====
                // === Driver Controller ===
                m_controllerDriver.button(ControllerDriverConstants.BUTTON_JOYSTICK_RIGHT)
                                .onTrue(Commands.parallel(
                                                Commands.runOnce(() -> m_elevator
                                                                .setGoalState(ElevatorState.CORAL_INTAKE)),
                                                Commands.runOnce(() -> m_drivebase
                                                                .setWantedSpeedState(SpeedState.NORMAL))));

                // === Operator Controller ===
                m_controllerOperator.button(ControllerOperatorConstants.BUTTON_A)
                                .onTrue(Commands.parallel(
                                                Commands.runOnce(() -> m_elevator
                                                                .setGoalState(ElevatorState.CORAL_INTAKE)),
                                                Commands.runOnce(() -> m_drivebase
                                                                .setWantedSpeedState(SpeedState.NORMAL))));

                m_controllerOperator.button(ControllerOperatorConstants.BUTTON_B)
                                .onTrue(Commands.parallel(
                                                Commands.runOnce(() -> m_elevator
                                                                .setGoalState(ElevatorState.CORAL_LEVEL_TWO)),
                                                Commands.runOnce(() -> m_drivebase
                                                                .setWantedSpeedState(SpeedState.SLOW))));

                m_controllerOperator.button(ControllerOperatorConstants.BUTTON_Y)
                                .onTrue(Commands.parallel(
                                                Commands.runOnce(() -> m_elevator
                                                                .setGoalState(ElevatorState.CORAL_LEVEL_THREE)),
                                                Commands.runOnce(() -> m_drivebase
                                                                .setWantedSpeedState(SpeedState.SLOW))));

                m_controllerOperator.button(ControllerOperatorConstants.BUTTON_X)
                                .onTrue(Commands.parallel(
                                                Commands.runOnce(() -> m_elevator
                                                                .setGoalState(ElevatorState.CORAL_LEVEL_FOUR)),
                                                Commands.runOnce(() -> m_drivebase
                                                                .setWantedSpeedState(SpeedState.SLOW))));

                // ===== Coral =====
                // === Operator Controller ===
                m_controllerOperator.button(ControllerOperatorConstants.BUTTON_BUMPER_TOP_LEFT)
                                .whileTrue(new IntakeCoralCommand(m_coral));

                m_controllerOperator.button(ControllerOperatorConstants.BUTTON_BUMPER_TOP_RIGHT)
                                .whileTrue(m_coral.manualForwardFastCommand());

                m_controllerOperator.pov(0)
                                .whileTrue(m_coral.manualForwardSlowCommand());

                m_controllerOperator.pov(180)
                                .whileTrue(m_coral.manualReverseSlowCommand());
                // ===== Flap =====
                // === Operator Controller ===
                switch (FlapConstants.FLAP_CONTROL_MODE) {
                        case ANGLE:
                                m_controllerOperator.button(ControllerOperatorConstants.BUTTON_BUMPER_BOTTOM_LEFT)
                                                .onTrue(Commands.runOnce(m_flap::stateShiftDown));

                                m_controllerOperator.button(ControllerOperatorConstants.BUTTON_BUMPER_BOTTOM_RIGHT)
                                                .onTrue(Commands.runOnce(m_flap::stateShiftUp));
                                break;
                        case MANUAL:
                                m_controllerOperator.button(ControllerOperatorConstants.BUTTON_BUMPER_BOTTOM_LEFT)
                                                .whileTrue(Commands.run(() -> m_flap.setWantedState(FlapState.DOWN)));

                                m_controllerOperator.button(ControllerOperatorConstants.BUTTON_BUMPER_BOTTOM_RIGHT)
                                                .whileTrue(Commands.run(() -> m_flap.setWantedState(FlapState.UP)));
                                break;
                }

                // ===== Climber =====
                // === Driver Controller ===
                m_controllerDriver.button(ControllerDriverConstants.BUTTON_BUMPER_BOTTOM_RIGHT)
                                .whileTrue(Commands.run(() -> m_climber.climbUp()));

                m_controllerDriver.button(ControllerDriverConstants.BUTTON_BUMPER_BOTTOM_LEFT)
                                .whileTrue(Commands.run(() -> m_climber.climbDown()));
                // === Operator Controller ===
                m_controllerOperator.pov(90)
                                .whileTrue(Commands.run(() -> m_climber.climbUp()));

                m_controllerOperator.pov(270)
                                .whileTrue(Commands.run(() -> m_climber.climbDown()));

                switch (CodeConstants.DEV_CONTROLLER_MODE) {
                        case ON:
                                m_controllerDeveloper.button(2)
                                                .onTrue(Commands.runOnce(
                                                                () -> m_elevator.setGoalState(ElevatorState.TEST)));

                                m_controllerDeveloper.button(5)
                                                .onTrue(Commands.runOnce(() -> m_elevator.decreaseTestHeight()));

                                m_controllerDeveloper.button(6)
                                                .onTrue(Commands.runOnce(() -> m_elevator.increaseTestHeight()));

                                break;
                        case OFF:
                                break;
                }
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An example command will be run in autonomous
                return m_chooser.getSelected();
        }

        public void setMotorBrake(boolean brake) {
                m_drivebase.setMotorBrake(brake);
        }
}
