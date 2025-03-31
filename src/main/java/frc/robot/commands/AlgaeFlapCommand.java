package frc.robot.commands;

// WPILib
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

// Subsystems
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.flap.FlapSubsystem;
import frc.robot.subsystems.flap.FlapSubsystem.FlapState;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// Constants
import frc.robot.Constants.CIAAutoConstants;

/**
 * Command to coordinate scoring a coral using elevator, flap, and drivebase.
 *
 * Sequence:
 * 1. Raise elevator and flap to UP.
 * 2. Level the flap while elevator holds position.
 * 3. Flip flap back UP briefly.
 * 4. Drive backward to release coral and finish.
 */
public class AlgaeFlapCommand extends Command {

    // Subsystem references
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final FlapSubsystem m_flapSubsystem;
    private final SwerveSubsystem m_drivebase;

    // Target elevator position for scoring
    private final ElevatorState m_elevatorLevel;

    // Internal state tracking
    private boolean m_commandDone = false;
    private final Timer m_timer = new Timer();

    /**
     * Enum to represent the sequence of the scoring routine.
     */
    private enum STAGE {
        STAGE_ONE, // Raise elevator and flap
        STAGE_TWO, // Level flap to release
        STAGE_THREE, // Return flap to UP
        STAGE_FOUR // Drive backward to finish
    }

    private STAGE m_currentStage;

    /**
     * Constructor to run the full coral scoring sequence using flap and drivebase.
     *
     * @param elevatorSubsystem Elevator used to raise coral
     * @param flapSubsystem     Flap that tips coral out
     * @param drivebase         Drivebase to back out after scoring
     * @param elevatorLevel     Target height to score from
     */
    public AlgaeFlapCommand(ElevatorSubsystem elevatorSubsystem, FlapSubsystem flapSubsystem,
            SwerveSubsystem drivebase, ElevatorState elevatorLevel) {
        this.m_elevatorSubsystem = elevatorSubsystem;
        this.m_flapSubsystem = flapSubsystem;
        this.m_drivebase = drivebase;
        this.m_elevatorLevel = elevatorLevel;

        // Prevent other commands from interfering with these subsystems
        addRequirements(m_elevatorSubsystem, m_flapSubsystem, m_drivebase);
    }

    /**
     * Called when the command is first scheduled.
     */
    @Override
    public void initialize() {
        m_currentStage = STAGE.STAGE_ONE;
        m_commandDone = false;

        m_timer.reset();
        m_timer.start();

        System.out.println("ALGAE FLAP COMMAND STARTED: Elevator Level - " + m_elevatorLevel.toString());
        System.out.println("Current Stage: " + m_currentStage.toString());
    }

    /**
     * Called every 20ms while the command is active.
     * Steps through each stage using the timer to control timing.
     */
    @Override
    public void execute() {
        switch (m_currentStage) {

            case STAGE_ONE:
                // Wait 1 second while raising elevator and flap
                if (m_timer.get() > 1.0) {
                    m_currentStage = STAGE.STAGE_TWO;
                    System.out.println("Current Stage: " + m_currentStage.toString());
                    m_timer.restart();
                } else {
                    m_elevatorSubsystem.setGoalState(m_elevatorLevel);
                    m_flapSubsystem.setWantedState(FlapState.UP);
                }
                break;

            case STAGE_TWO:
                // Wait 1 second while leveling flap to eject coral
                if (m_timer.get() > 1.0) {
                    m_currentStage = STAGE.STAGE_THREE;
                    System.out.println("Current Stage: " + m_currentStage.toString());
                    m_timer.restart();
                } else {
                    m_elevatorSubsystem.setGoalState(m_elevatorLevel);
                    m_flapSubsystem.setWantedState(FlapState.LEVEL);
                }
                break;

            case STAGE_THREE:
                // Flip flap back UP after short delay
                if (m_timer.get() > 0.5) {
                    m_currentStage = STAGE.STAGE_FOUR;
                    System.out.println("Current Stage: " + m_currentStage.toString());
                    m_timer.restart();
                } else {
                    m_elevatorSubsystem.setGoalState(m_elevatorLevel);
                    m_flapSubsystem.setWantedState(FlapState.UP);
                }
                break;

            case STAGE_FOUR:
                // Drive backward for 0.5 seconds, then complete
                if (m_timer.get() > 0.5) {
                    m_commandDone = true;
                } else {
                    m_elevatorSubsystem.setGoalState(m_elevatorLevel);
                    m_flapSubsystem.setWantedState(FlapState.UP);
                    m_drivebase.drive(new ChassisSpeeds(-5.0, 0.0, 0.0)); // Reverse drive
                }
                break;
        }
    }

    /**
     * Called once the command ends (naturally or by interruption).
     * Resets elevator to coral intake position.
     */
    @Override
    public void end(boolean interrupted) {
        System.out.println("ALGAE FLAP COMMAND COMPLETE" + (interrupted ? " (INTERRUPTED)" : ""));
        m_elevatorSubsystem.setGoalState(ElevatorState.CORAL_INTAKE); // Return to default
    }

    /**
     * Returns true when the full sequence has finished.
     */
    @Override
    public boolean isFinished() {
        return m_commandDone;
    }
}
