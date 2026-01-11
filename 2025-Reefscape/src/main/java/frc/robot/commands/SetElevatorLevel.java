package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorLevel extends Command{
    private final ElevatorSubsystem m_subsystem;
    private int m_setIndex;


    /**
     * Creates a new CoralSpeedCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public SetElevatorLevel(ElevatorSubsystem subsystem, int setIndex) 
    {
        m_subsystem = subsystem;
        m_setIndex = setIndex;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.setElevatorLevel(m_setIndex);
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
