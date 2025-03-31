package frc.robot.subsystems.coral;

// Java standard library
import java.util.function.BooleanSupplier;

// WPILib imports
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// REV Robotics SparkMax imports
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

// Project-specific imports
import frc.robot.Constants.CoralConstants;

/**
 * Subsystem responsible for controlling the "Coral" intake mechanism.
 * Includes dual motors (left/right) and a digital input sensor for detecting
 * objects.
 */
public class CoralSubsystem extends SubsystemBase {

    // Left and right motors for the coral intake
    private final SparkMax m_motorLeft, m_motorRight;
    // Digital input sensor used to detect game pieces or intake state
    private final DigitalInput m_intakeSensor;

    // Stores last read sensor value for SmartDashboard reporting
    private boolean m_intakeSensorValue = false;

    public CoralSubsystem() {
        // Initialize motor controllers with CAN IDs and types
        m_motorLeft = new SparkMax(CoralConstants.MOTOR_LEFT_CANID, CoralConstants.MOTOR_LEFT_TYPE);
        m_motorRight = new SparkMax(CoralConstants.MOTOR_RIGHT_CANID, CoralConstants.MOTOR_RIGHT_TYPE);

        // Configure left motor
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig
                .inverted(CoralConstants.MOTOR_LEFT_IS_INVERTED); // Set inversion if needed
        m_motorLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure right motor to follow the left motor
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig
                .follow(m_motorLeft, CoralConstants.MOTOR_RIGHT_IS_INVERTED); // Follow with optional inversion
        m_motorRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize digital sensor on DIO port
        m_intakeSensor = new DigitalInput(CoralConstants.INTAKE_SENSOR_DIOPORT);

        // Set default behavior of subsystem to stop the motor
        setDefaultCommand(run(this::stop).withName("DEFAULT: STOPPED"));
    }

    @Override
    public void periodic() {
        // Update cached sensor value each loop
        m_intakeSensorValue = getIntakeSensorValue();
        // Push values to SmartDashboard for driver monitoring
        sendSmartDashboardValues();
    }

    /** Sends the current intake sensor state to the SmartDashboard. */
    private void sendSmartDashboardValues() {
        SmartDashboard.putBoolean("Coral Intake Sensor", m_intakeSensorValue);
    }

    /**
     * Returns the current value of the intake sensor.
     * Can be inverted depending on wiring/config.
     */
    public boolean getIntakeSensorValue() {
        return CoralConstants.INTAKE_SENSOR_IS_INVERTED ? !m_intakeSensor.get() : m_intakeSensor.get();
    }

    /**
     * Returns a BooleanSupplier version of the intake sensor,
     * allowing it to be used in command-based triggers.
     */
    public BooleanSupplier getIntakeSensorSupplier() {
        return this::getIntakeSensorValue;
    }

    /**
     * Sets the intake motor (left + followed right) speed.
     * 
     * @param newSpeed Desired percent output speed [-1.0, 1.0]
     */
    public void setSpeed(double newSpeed) {
        m_motorLeft.set(newSpeed);
    }

    /**
     * Stops the intake motor.
     */
    public void stop() {
        m_motorLeft.stopMotor();
    }

    // === Command Factories ===

    /** Returns a command that stops the coral subsystem. */
    public Command stoppedCommand() {
        return run(this::stop).withName("STOPPED");
    }

    /** Returns a command that runs coral forward slowly. */
    public Command manualForwardSlowCommand() {
        return run(() -> setSpeed(CoralConstants.SPEED_MANUAL_FORWARD_SLOW)).withName("MANUAL_FORWARD_SLOW");
    }

    /** Returns a command that runs coral forward quickly. */
    public Command manualForwardFastCommand() {
        return run(() -> setSpeed(CoralConstants.SPEED_MANUAL_FORWARD_FAST)).withName("MANUAL_FORWARD_FAST");
    }

    /** Returns a command that runs coral in reverse slowly. */
    public Command manualReverseSlowCommand() {
        return run(() -> setSpeed(CoralConstants.SPEED_MANUAL_REVERSE_SLOW)).withName("MANUAL_REVERSE_SLOW");
    }

    /** Returns a command that runs coral in reverse quickly. */
    public Command manualReverseFastCommand() {
        return run(() -> setSpeed(CoralConstants.SPEED_MANUAL_REVERSE_FAST)).withName("MANUAL_REVERSE_FAST");
    }
}
