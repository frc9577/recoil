package frc.robot.commands.autoCommands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AimAtHub;
import frc.robot.commands.util.AutoFromList;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class CorralAndShoot extends Command {
  // The sequance of the command, any "*" are place holders for commands.
  private final ArrayList<Object> m_baseSequence = new ArrayList<Object>(Arrays.asList(
    "GatherCorralShootVel", 
    "*"  // Aim at hub
  ));

  // Pass-ins
  private final DriveSubsystem m_DriveSubsystem;
  private final DifferentialDrivePoseEstimator m_PoseEstimator;
  private final BooleanSupplier m_isRed;
  private final PathConstraints m_constraints;

  /**
   * Creates a new CorralAndShoot.
   */
  public CorralAndShoot(DriveSubsystem driveSubsystem, DifferentialDrivePoseEstimator poseEstimator, BooleanSupplier isRed, PathConstraints constraints) {
    m_DriveSubsystem = driveSubsystem;
    m_PoseEstimator = poseEstimator;
    m_isRed = isRed;
    m_constraints = constraints;
    
    // warm-up for quick load later on
    for (Object item : m_baseSequence) {
        if (item instanceof String && (String) item != "*") {
          try {
              PathPlannerPath path = PathPlannerPath.fromPathFile((String) item);
              Command command = AutoBuilder.followPath(path);
              new PathPlannerAuto(command);
          } catch (Exception e) {
              System.out.println(e);
          }
        }
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Init the new sequence
    ArrayList<Object> sequence = new ArrayList<Object>(m_baseSequence);
    sequence.set(1, new AimAtHub(m_DriveSubsystem, m_PoseEstimator, 2.0, m_isRed));

    // Run the command
    Command auto = new AutoFromList(sequence, m_constraints, m_DriveSubsystem, m_PoseEstimator, true, false);
    CommandScheduler.getInstance().schedule(auto);
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
