package frc.robot.commands;

// WPILib imports
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

// Project-specific imports
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.Constants.CIAAutoConstants;

/**
 * Command for automatically intaking a coral (game piece).
 * 
 * Staged logic:
 * 1. Intake runs forward until sensor is tripped (object enters).
 * 2. Intake speed adjusts while object is inside.
 * 3. Intake reverses briefly after sensor clears (object exits), then stops.
 */
public class IntakeCoralCommand extends Command {

    private final CoralSubsystem m_coralSubsystem;
    private boolean m_commandDone;
    private final Timer m_timer = new Timer();

    // Defines stages of coral intake logic
    private enum STAGE {
        STAGE_ONE, // Waiting for coral to enter
        STAGE_TWO, // Coral is inside
        STAGE_THREE // Coral has exited
    }

    private STAGE m_currentStage;

    public IntakeCoralCommand(CoralSubsystem coralSubsystem) {
        this.m_coralSubsystem = coralSubsystem;
        addRequirements(coralSubsystem); // Declare exclusive use of CoralSubsystem
    }

    @Override
    public void initialize() {
        m_currentStage = STAGE.STAGE_ONE;
        m_commandDone = false;
        m_timer.reset();

        System.out.println("INTAKE CORAL COMMAND STARTED");
        System.out.println("Current Stage: " + m_currentStage.toString());
    }

    @Override
    public void execute() {
        switch (m_currentStage) {

            case STAGE_ONE:
                // Wait for coral to trigger sensor
                if (m_coralSubsystem.getIntakeSensorValue()) {
                    m_currentStage = STAGE.STAGE_TWO;
                    System.out.println("Current Stage: " + m_currentStage.toString());
                } else {
                    m_coralSubsystem.setSpeed(CIAAutoConstants.AUTO_SPEED_CORAL_BEFORE_ENTER);
                }
                break;

            case STAGE_TWO:
                // Wait for coral to exit (sensor no longer triggered)
                if (!m_coralSubsystem.getIntakeSensorValue()) {
                    m_currentStage = STAGE.STAGE_THREE;
                    m_timer.restart();
                    System.out.println("Current Stage: " + m_currentStage.toString());
                } else {
                    m_coralSubsystem.setSpeed(CIAAutoConstants.AUTO_SPEED_CORAL_AFTER_ENTER);
                }
                break;

            case STAGE_THREE:
                // Run reverse briefly, then check if coral came back (abort condition)
                if (m_coralSubsystem.getIntakeSensorValue()) {
                    // Possibly re-ingested — abort intake process
                    m_commandDone = true;
                } else {
                    // Brief reverse to make sure coral clears
                    m_coralSubsystem.setSpeed(-CIAAutoConstants.AUTO_SPEED_CORAL_AFTER_ENTER);

                    // Optional: time limit before automatically ending
                    if (m_timer.hasElapsed(1.0)) {
                        m_commandDone = true;
                    }
                }
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("INTAKE CORAL COMMAND COMPLETE" + (interrupted ? " (INTERRUPTED)" : ""));
        m_coralSubsystem.setSpeed(0.0); // Stop intake motor
    }

    @Override
    public boolean isFinished() {
        return m_commandDone;
    }
}
