package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;

public class DifferentailDriveFromDashboard extends Command {
    private final DriveSubsystem m_subsystem;

    /**
     * Creates a new ArcadeDriveCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public DifferentailDriveFromDashboard(DriveSubsystem subsystem, CommandXboxController driveController)  {
        m_subsystem = subsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double leftSpeed  = SmartDashboard.getNumber("DB Left Set (MPS)", 0);
        double rightSpeed = SmartDashboard.getNumber("DB Right Set (MPS)", 0);

        m_subsystem.setDifferentialSpeeds( leftSpeed, rightSpeed );
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
