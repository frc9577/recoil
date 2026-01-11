package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ChangeElevatorLevel extends Command{
    private final ElevatorSubsystem m_subsystem;
    private int m_changeInLevel;


    /**
     * Creates a new CoralSpeedCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ChangeElevatorLevel(ElevatorSubsystem subsystem, int changeInLevel) 
    {
        m_subsystem = subsystem;
        m_changeInLevel = changeInLevel;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.changeElevatorLevel(m_changeInLevel);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
