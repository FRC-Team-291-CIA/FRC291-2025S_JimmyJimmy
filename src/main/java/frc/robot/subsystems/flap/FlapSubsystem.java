package frc.robot.subsystems.flap;

// WPILib imports
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// REV Robotics SparkMax imports
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;

// Project-specific Imports
import frc.robot.Constants.FlapConstants;

/**
 * Subsystem to control the robot's flap mechanism, including closed-loop
 * position control.
 */
public class FlapSubsystem extends SubsystemBase {

    private final SparkMax m_motor;
    private final AbsoluteEncoder m_encoder;
    private final SparkClosedLoopController m_controller;

    // Latest recorded flap angle, updated periodically
    private double m_flapAngle = -291.0;

    /**
     * Enum representing possible flap positions or states.
     * Each state maps to a target angle.
     */
    public enum FlapState {
        UP(FlapConstants.ANGLE_UP),
        LEVEL(FlapConstants.ANGLE_LEVEL),
        DOWN(FlapConstants.ANGLE_DOWN),
        DISABLED(FlapConstants.ANGLE_DISABLED);

        private final double angle;

        FlapState(double angle) {
            this.angle = angle;
        }

        public double getAngle() {
            return angle;
        }
    }

    // Current desired flap state
    private FlapState m_currentFlapState = FlapState.DISABLED;

    public FlapSubsystem() {
        // Initialize SparkMax motor with CAN ID and motor type
        m_motor = new SparkMax(FlapConstants.MOTOR_CANID, FlapConstants.MOTOR_TYPE);

        // Set up motor configuration
        SparkMaxConfig config = new SparkMaxConfig();

        config
                .inverted(FlapConstants.MOTOR_IS_INVERTED)
                .smartCurrentLimit(FlapConstants.MOTOR_SMART_CURRENT_LIMIT)
                .idleMode(FlapConstants.MOTOR_MODE);

        // Set encoder to return angles in degrees (0–360)
        config.absoluteEncoder.positionConversionFactor(360);

        // Set closed-loop PID and feedback configuration
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(FlapConstants.SLOT_ZERO_P, ClosedLoopSlot.kSlot0)
                .i(FlapConstants.SLOT_ZERO_I, ClosedLoopSlot.kSlot0)
                .d(FlapConstants.SLOT_ZERO_D, ClosedLoopSlot.kSlot0)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot0);

        // Apply configuration to motor controller
        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Get encoder and PID controller references from the SparkMax
        m_encoder = m_motor.getAbsoluteEncoder();
        m_controller = m_motor.getClosedLoopController();

        // Initialize angle from encoder
        m_flapAngle = m_encoder.getPosition();

        // Set default command to hold current state
        switch (FlapConstants.FLAP_CONTROL_MODE) {
            case ANGLE:
                setDefaultCommand(run(() -> setWantedState(m_currentFlapState))
                        .withName("DEFAULT: " + m_currentFlapState.toString()));
                break;
            case MANUAL:
                setDefaultCommand(run(() -> m_motor.stopMotor())
                        .withName("DEFAULT: " + FlapState.DISABLED.toString()));
                break;
        }
    }

    @Override
    public void periodic() {
        // Update the flap angle every cycle
        m_flapAngle = m_encoder.getPosition();
        sendSmartDashboardValues();
    }

    /**
     * Sends current flap angle to SmartDashboard for monitoring.
     */
    private void sendSmartDashboardValues() {
        SmartDashboard.putNumber("Flap Angle:", m_flapAngle);
    }

    /**
     * Set the desired flap state.
     * Uses PID control if in ANGLE mode, or direct motor commands in MANUAL mode.
     */
    public void setWantedState(FlapState targetState) {
        m_currentFlapState = targetState;

        // Only allow flap movement if robot is enabled and target state is valid
        boolean robotEnabled = DriverStation.isEnabled();
        boolean validState = targetState != FlapState.DISABLED;

        if (robotEnabled && validState) {
            switch (FlapConstants.FLAP_CONTROL_MODE) {
                case ANGLE:
                    // Use PID position control
                    m_controller.setReference(
                            targetState.getAngle(),
                            ControlType.kPosition,
                            ClosedLoopSlot.kSlot0,
                            FlapConstants.SLOT_ZERO_FF,
                            FlapConstants.SLOT_ZERO_FF_UNITS);
                    break;
                case MANUAL:
                    // Use direct motor output for up/down; stop otherwise
                    switch (targetState) {
                        case DOWN -> m_motor.set(FlapConstants.DOWN_SPEED);
                        case UP -> m_motor.set(FlapConstants.UP_SPEED);
                        default -> m_motor.stopMotor();
                    }
                    break;
            }
        } else {
            m_motor.stopMotor();
        }
    }

    // FRC 291: Add Shift Toggles for Speed
    public void stateShiftUp() {
        switch (m_currentFlapState) {
            case UP:
                // State at Max. Do Nothing.
                break;
            case LEVEL:
                this.setWantedState(FlapState.UP);
                break;
            case DOWN:
                this.setWantedState(FlapState.LEVEL);
                break;
            case DISABLED:
                // If Disabled Assumed in Down State
                this.setWantedState(FlapState.LEVEL);
                break;
        }
    }

    public void stateShiftDown() {
        switch (m_currentFlapState) {
            case UP:
                this.setWantedState(FlapState.LEVEL);
                break;
            case LEVEL:
                this.setWantedState(FlapState.DOWN);
                break;
            case DOWN:
                // State at Min. Do Nothing.
                break;
            case DISABLED:
                // If Disabled Assumed in Down State
                this.setWantedState(FlapState.LEVEL);
                break;
        }
    }
}
