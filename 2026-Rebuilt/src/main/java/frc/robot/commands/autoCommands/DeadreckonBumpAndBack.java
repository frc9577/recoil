package frc.robot.commands.autoCommands;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AimAtHub;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.PathUtils;
import frc.robot.utils.Pigeon;

/** An example command that uses an example subsystem. */
public class DeadreckonBumpAndBack extends Command {
  private final DriveSubsystem m_driveSubsystem;
  private final DifferentialDrivePoseEstimator m_poseEstimator;
  private final PathConstraints m_constraints;
  private final BooleanSupplier m_isRed;
  private final Pigeon m_pigeon;

  /**
   * Creates a new DeadreckonBumpAndBack.
   *
   * @param poseEstimator The pose estimator to get the location of the robot.
   * @param targetPose The pose the robot will make a point to go to.
   * @param constraints The constraints the robot will follow while following the path. 
   */
  public DeadreckonBumpAndBack(DriveSubsystem driveSubsystem, DifferentialDrivePoseEstimator poseEstimator, PathConstraints constraints, BooleanSupplier isRed, Pigeon pigeon) 
  {
    m_driveSubsystem = driveSubsystem;
    m_poseEstimator = poseEstimator;
    m_constraints = constraints;
    m_isRed = isRed;
    m_pigeon = pigeon;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Rotation2d targetRotation;
    Double pickupTargetX;
    Double lineupTargetX;
    if (m_isRed.getAsBoolean() == true) { // Red
      targetRotation = Rotation2d.k180deg;

      pickupTargetX = 9.1;
      lineupTargetX = 10.88;
    } else { // Blue
      targetRotation = Rotation2d.kZero;

      pickupTargetX = 7.689;
      lineupTargetX = 5.667;
    }

    // Deadreckon Over the bump
    Command deadreckonFoward = new DeadreckonDistance(m_driveSubsystem, 2.5, 2.0);
    Command deadreckonBackward = new DeadreckonDistance(m_driveSubsystem, 2.5, -2.0);

    // Create paths and scheudle other commands
    Command lineupSchedule = new InstantCommand(() -> {
      Pose2d currentPose = m_poseEstimator.getEstimatedPosition();
      Rotation2d currentRotation = currentPose.getRotation();
      Double currentY = currentPose.getY();

      Pose2d lineupTarget = new Pose2d(lineupTargetX, currentY, currentRotation);

      System.out.println(lineupTarget.toString());

      // Go to the lineup target (backward)
      PathPlannerPath lineupPath = PathUtils.OnTheFlyToTarget(
        currentPose, 
        lineupTarget, 
        m_constraints, 
        true
      );
      Command lineupCommand = AutoBuilder.followPath(lineupPath);

      // Create & Schedule the command group.
      CommandScheduler.getInstance().schedule(
        lineupCommand
        .andThen(deadreckonBackward)
        .andThen(new AimAtHub(m_driveSubsystem, m_poseEstimator, 2.0, m_isRed))
        .andThen(new DeadreckonDistance(m_driveSubsystem, 0.75, -1.0))
        .andThen(new AimAtHub(m_driveSubsystem, m_poseEstimator, 2.0, m_isRed))
      );
    });

    Command pickupScheudle = new InstantCommand(() -> {
      Pose2d currentPose = m_poseEstimator.getEstimatedPosition();
      Rotation2d currentRotation = currentPose.getRotation();
      Double currentY = currentPose.getY();

      Pose2d pickupTarget = new Pose2d(pickupTargetX, currentY, currentRotation);

      System.out.println(pickupTarget.toString());

      // Go to the pickup target (forward)
      PathPlannerPath pickupPath = PathUtils.OnTheFlyToTarget(currentPose, pickupTarget, m_constraints);
      Command pickupCommand = AutoBuilder.followPath(pickupPath);

      // Create & Scheudle the command group
      CommandScheduler.getInstance().schedule(pickupCommand.andThen(lineupSchedule));
    });
    
    // Create & Schedule the command group.
    CommandScheduler.getInstance().schedule(
      deadreckonFoward
      .andThen(pickupScheudle)
    );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // The command automatically ends
    return true;
  }
}
