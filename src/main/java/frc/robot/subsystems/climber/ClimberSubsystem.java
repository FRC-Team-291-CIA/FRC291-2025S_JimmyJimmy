package frc.robot.subsystems.climber;

// WPILib imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// REV Robotics SparkMax imports
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

// Project-specific imports
import frc.robot.Constants.ClimberConstants;

/**
 * Subsystem class for controlling the robot's climber mechanism.
 * Utilizes a REV SparkMax motor controller.
 */
public class ClimberSubsystem extends SubsystemBase {

    // Motor controller for the climber
    private final SparkMax m_motor;

    public ClimberSubsystem() {
        // Initialize the SparkMax motor with CAN ID and motor type from constants
        m_motor = new SparkMax(ClimberConstants.MOTOR_CANID, ClimberConstants.MOTOR_TYPE);

        // Create and configure SparkMax settings
        SparkMaxConfig config = new SparkMaxConfig();

        config
                // Set motor direction based on whether it needs to be inverted
                .inverted(ClimberConstants.MOTOR_IS_INVERTED)
                // Limit the amount of current the motor can draw to protect hardware
                .smartCurrentLimit(ClimberConstants.MOTOR_SMART_CURRENT_LIMIT)
                // Set idle behavior (coast or brake) when not powered
                .idleMode(ClimberConstants.MOTOR_MODE);

        // Apply configuration with safe reset and persistent parameters
        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Set default command to stop the climber motor when no other commands are
        // running
        setDefaultCommand(run(this::stop).withName("DEFAULT: STOPPED"));
    }

    @Override
    public void periodic() {
        // Called every scheduler run; used here to send telemetry
        sendSmartDashboardValues();
    }

    /**
     * Sends real-time telemetry values to the SmartDashboard for driver/staff
     * monitoring.
     */
    private void sendSmartDashboardValues() {
        // Output current draw of the motor
        SmartDashboard.putNumber("Climber Motor Current", m_motor.getOutputCurrent());
    }

    /**
     * Stops the climber motor. Useful for halting movement or default state.
     */
    public void stop() {
        m_motor.stopMotor();
    }

    /**
     * Runs the climber motor inward, e.g., pulling the climber up.
     */
    public void climbUp() {
        m_motor.set(ClimberConstants.SPEED_IN);
    }

    /**
     * Runs the climber motor outward, e.g., extending or climbing down.
     */
    public void climbDown() {
        m_motor.set(ClimberConstants.SPEED_OUT);
    }
}
