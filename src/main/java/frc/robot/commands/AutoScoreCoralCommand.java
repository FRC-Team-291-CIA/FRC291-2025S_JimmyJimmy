package frc.robot.commands;

// === WPILib ===
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

// === Subsystems ===
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorState;

// === Constants ===
import frc.robot.Constants.CIAAutoConstants;

/**
 * Command to score a coral (game piece) using the intake and elevator
 * subsystems.
 *
 * Sequence:
 * 1. Move elevator to scoring position for a fixed time.
 * 2. Run the coral intake briefly to eject the piece.
 */
public class AutoScoreCoralCommand extends Command {

    private final CoralSubsystem m_coralSubsystem;
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final ElevatorState m_elevatorLevel;

    private boolean m_commandDone = false;
    private final Timer m_timer = new Timer();

    // Internal staged state for sequencing elevator and intake logic
    private enum STAGE {
        STAGE_ONE, // Move elevator
        STAGE_TWO // Run intake to score
    }

    private STAGE m_currentStage;

    /**
     * Constructor for the coral scoring command.
     *
     * @param coralSubsystem    Subsystem that controls the intake
     * @param elevatorSubsystem Subsystem that positions the elevator
     * @param elevatorLevel     The desired scoring height
     */
    public AutoScoreCoralCommand(CoralSubsystem coralSubsystem, ElevatorSubsystem elevatorSubsystem,
            ElevatorState elevatorLevel) {
        this.m_coralSubsystem = coralSubsystem;
        this.m_elevatorSubsystem = elevatorSubsystem;
        this.m_elevatorLevel = elevatorLevel;

        // Declare resource requirements
        addRequirements(coralSubsystem, elevatorSubsystem);
    }

    /**
     * Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        m_currentStage = STAGE.STAGE_ONE;
        m_commandDone = false;

        m_timer.reset();
        m_timer.start();

        System.out.println("SCORE CORAL COMMAND STARTED -> " + m_elevatorLevel.toString());
    }

    /**
     * Called every 20ms while the command is scheduled.
     */
    @Override
    public void execute() {
        switch (m_currentStage) {

            case STAGE_ONE:
                // Give elevator time to reach scoring height
                if (m_timer.hasElapsed(2.0)) {
                    m_currentStage = STAGE.STAGE_TWO;
                    System.out.println("Transitioned to: " + m_currentStage.toString());
                    m_timer.restart();
                } else {
                    m_elevatorSubsystem.setGoalState(m_elevatorLevel);
                }
                break;

            case STAGE_TWO:
                // Run intake to score, then finish
                if (m_timer.hasElapsed(3)) {
                    m_commandDone = true;
                } else {
                    m_elevatorSubsystem.setGoalState(m_elevatorLevel); // Keep holding position
                    m_coralSubsystem.setSpeed(CIAAutoConstants.AUTO_SCORE_SPEED_OUT); // Score coral
                }
                break;
        }
    }

    /**
     * Called when the command ends or is interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        System.out.println("SCORE CORAL COMMAND COMPLETE" + (interrupted ? " (INTERRUPTED)" : ""));
        m_coralSubsystem.setSpeed(0.0); // Always stop intake motor
        m_elevatorSubsystem.setGoalState(ElevatorState.CORAL_INTAKE); // Return elevator to safe state
    }

    /**
     * Whether this command is finished.
     */
    @Override
    public boolean isFinished() {
        return m_commandDone;
    }
}
