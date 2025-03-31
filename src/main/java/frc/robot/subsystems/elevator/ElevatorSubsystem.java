package frc.robot.subsystems.elevator;

// WPILib imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// REV Robotics SparkMax imports
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

// Project-specific Imports
import frc.robot.Constants.ElevatorConstants;

/**
 * Subsystem controlling a dual-motor elevator mechanism using SparkMax with PID
 * control.
 */
public class ElevatorSubsystem extends SubsystemBase {

    // Motor controllers
    private final SparkMax m_motorLeft, m_motorRight;

    // Encoder and controller on the lead motor
    private final RelativeEncoder m_encoderLeft;
    private final SparkClosedLoopController m_controllerLeft;

    // Latest elevator position (in inches)
    private double m_elevatorHeight = -291.0;

    /**
     * Enum representing logical target positions for the elevator.
     */
    public enum ElevatorState {
        CORAL_LEVEL_FOUR(ElevatorConstants.HEIGHT_CORAL_LEVEL_FOUR),
        CORAL_LEVEL_THREE(ElevatorConstants.HEIGHT_CORAL_LEVEL_THREE),
        CORAL_LEVEL_TWO(ElevatorConstants.HEIGHT_CORAL_LEVEL_TWO),
        CORAL_LEVEL_ONE(ElevatorConstants.HEIGHT_CORAL_LEVEL_ONE),
        CORAL_INTAKE(ElevatorConstants.HEIGHT_CORAL_INTAKE),
        PARK(ElevatorConstants.HEIGHT_PARK),
        NO_POWER(ElevatorConstants.HEIGHT_KILL_POWER),
        DISABLED(ElevatorConstants.HEIGHT_DISABLED);

        private final double height;

        ElevatorState(double height) {
            this.height = height;
        }

        public double getHeight() {
            return height;
        }
    }

    /**
     * Enum tracking motor PID configuration (UP vs DOWN),
     * because different feedforward values and PID slots may apply depending on
     * direction.
     */
    private enum MotorState {
        UP(ClosedLoopSlot.kSlot0, ElevatorConstants.SLOT_ZERO_FF_UNITS, ElevatorConstants.SLOT_ZERO_FF),
        DOWN(ClosedLoopSlot.kSlot1, ElevatorConstants.SLOT_ONE_FF_UNITS, ElevatorConstants.SLOT_ONE_FF);

        private final ClosedLoopSlot slot;
        private final ArbFFUnits units;
        private final double ff;

        MotorState(ClosedLoopSlot slot, ArbFFUnits units, double ff) {
            this.slot = slot;
            this.units = units;
            this.ff = ff;
        }

        public ClosedLoopSlot getSlot() {
            return slot;
        }

        public ArbFFUnits getUnits() {
            return units;
        }

        public double getFF() {
            return ff;
        }
    }

    public ElevatorState m_currentElevatorState = ElevatorState.DISABLED;
    public ElevatorState m_previousElevatorState = ElevatorState.DISABLED;
    private MotorState m_currentMotorState = MotorState.UP;

    public ElevatorSubsystem() {
        // Initialize both motors
        m_motorLeft = new SparkMax(ElevatorConstants.MOTOR_LEFT_CANID, ElevatorConstants.MOTOR_LEFT_TYPE);
        m_motorRight = new SparkMax(ElevatorConstants.MOTOR_RIGHT_CANID, ElevatorConstants.MOTOR_RIGHT_TYPE);

        // Configure left (primary) motor
        SparkMaxConfig leftConfig = new SparkMaxConfig();

        leftConfig
                .inverted(ElevatorConstants.MOTOR_LEFT_IS_INVERTED)
                .smartCurrentLimit(ElevatorConstants.MOTOR_SMART_CURRENT_LIMIT)
                .idleMode(ElevatorConstants.MOTOR_LEFT_MODE);

        leftConfig.encoder
                .positionConversionFactor(ElevatorConstants.MOTOR_REVOLUTION_PER_INCH)
                .velocityConversionFactor(ElevatorConstants.MOTOR_REVOLUTION_PER_INCH);

        leftConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // PID values for upward motion
                .p(ElevatorConstants.SLOT_ZERO_P, ClosedLoopSlot.kSlot0)
                .i(ElevatorConstants.SLOT_ZERO_I, ClosedLoopSlot.kSlot0)
                .d(ElevatorConstants.SLOT_ZERO_D, ClosedLoopSlot.kSlot0)
                // PID values for downward motion
                .p(ElevatorConstants.SLOT_ONE_P, ClosedLoopSlot.kSlot1)
                .i(ElevatorConstants.SLOT_ONE_I, ClosedLoopSlot.kSlot1)
                .d(ElevatorConstants.SLOT_ONE_D, ClosedLoopSlot.kSlot1)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        m_motorLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure right motor to follow left motor
        SparkMaxConfig rightConfig = new SparkMaxConfig();

        rightConfig
                .follow(m_motorLeft, ElevatorConstants.MOTOR_RIGHT_IS_INVERTED)
                .idleMode(ElevatorConstants.MOTOR_RIGHT_MODE);

        m_motorRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Get encoder and controller from the left motor
        m_encoderLeft = m_motorLeft.getEncoder();
        m_controllerLeft = m_motorLeft.getClosedLoopController();

        m_elevatorHeight = m_encoderLeft.getPosition();

        // Set default command to maintain current state
        setDefaultCommand(new RunCommand(() -> setGoalState(m_currentElevatorState), this));
    }

    @Override
    public void periodic() {
        // Update elevator height each loop
        m_elevatorHeight = m_encoderLeft.getPosition();
        sendSmartDashboardValues();
    }

    /**
     * Outputs current telemetry to SmartDashboard.
     */
    private void sendSmartDashboardValues() {
        SmartDashboard.putString("Elevator: State", m_currentElevatorState.toString());
        SmartDashboard.putString("Elevator: Motor Mode", m_currentMotorState.toString());
        SmartDashboard.putNumber("Elevator: Goal Height", m_currentElevatorState.getHeight());
        SmartDashboard.putNumber("Elevator: Current Height", m_elevatorHeight);
        SmartDashboard.putNumber("Elevator: Left Amps", m_motorLeft.getOutputCurrent());
        SmartDashboard.putNumber("Elevator: Right Amps", m_motorRight.getOutputCurrent());
    }

    /**
     * Changes the elevator to the desired target state.
     * Automatically adjusts PID slot/feedforward depending on direction.
     */
    public void setGoalState(ElevatorState targetState) {
        m_currentElevatorState = targetState;

        // Check for state change and update control mode if needed
        if (targetState != m_previousElevatorState) {
            updateMotorDirection();
            m_previousElevatorState = targetState;
        }

        // If no movement is wanted, stop the motor
        if (targetState == ElevatorState.DISABLED || targetState == ElevatorState.NO_POWER) {
            m_motorLeft.stopMotor();
            return;
        }

        // Set closed-loop position reference using current slot and FF
        m_controllerLeft.setReference(
                targetState.getHeight(), ControlType.kPosition,
                m_currentMotorState.getSlot(),
                m_currentMotorState.getFF(),
                m_currentMotorState.getUnits());
    }

    /**
     * Updates motor direction (UP or DOWN) based on current vs target height.
     */
    private void updateMotorDirection() {
        m_currentMotorState = (m_elevatorHeight <= m_currentElevatorState.getHeight())
                ? MotorState.DOWN
                : MotorState.UP;
    }
}
