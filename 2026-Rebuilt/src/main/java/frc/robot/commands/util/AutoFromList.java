package frc.robot.commands.util;

import java.io.Console;
import java.nio.file.Path;
import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RotateToRotation2D;
import frc.robot.subsystems.DriveSubsystem;


// Auto From Path Names And Commands
public class AutoFromList extends Command {
    private final List<Object> m_sequence;
    private final PathConstraints m_constraints;
    private final DriveSubsystem m_DriveSubsystem;
    private final DifferentialDrivePoseEstimator m_poseEstimator;
    private final Boolean m_pathfindToFirstPath;

    /**
     * Creates and runs a new auto from a list of path names and commands.
     * Any list with a command can only be ran once due to how the scheudler works.
     * 
     * @param sequence A list of path names (strings) and commands to run in order.
     */
    public AutoFromList(List<Object> sequence, PathConstraints constraints, DriveSubsystem driveSubsystem, DifferentialDrivePoseEstimator poseEstimator) 
    {
        m_sequence = sequence;
        m_constraints = constraints;
        m_DriveSubsystem = driveSubsystem;
        m_poseEstimator = poseEstimator;
        m_pathfindToFirstPath = true;

        warmup();
    }

    public AutoFromList(List<Object> sequence, PathConstraints constraints, DriveSubsystem driveSubsystem, DifferentialDrivePoseEstimator poseEstimator, boolean pathfindToFirstpath) 
    {
        m_sequence = sequence;
        m_constraints = constraints;
        m_DriveSubsystem = driveSubsystem;
        m_poseEstimator = poseEstimator;
        m_pathfindToFirstPath = pathfindToFirstpath;

        warmup();
    }

    public AutoFromList(List<Object> sequence, PathConstraints constraints, DriveSubsystem driveSubsystem, DifferentialDrivePoseEstimator poseEstimator, boolean pathfindToFirstpath, boolean doWarmup) 
    {
        m_sequence = sequence;
        m_constraints = constraints;
        m_DriveSubsystem = driveSubsystem;
        m_poseEstimator = poseEstimator;
        m_pathfindToFirstPath = pathfindToFirstpath;

        if (doWarmup == true) {
            warmup();
        }
    }

    private void warmup() {
        for (Object item : m_sequence) {
            if (item instanceof String) {
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

    @Override
    public void initialize() {
        try {
            SequentialCommandGroup precommandGroup = new SequentialCommandGroup();
            SequentialCommandGroup commandGroup = new SequentialCommandGroup();
            Boolean pathFinded = !m_pathfindToFirstPath; // If we want to pathfind this will be set to false.

            for (Object item : m_sequence) {
                if (item instanceof Command) {
                    if (pathFinded == false) {
                        precommandGroup.addCommands((Command) item);
                    } else {
                        commandGroup.addCommands((Command) item);
                    }
                } else if (item instanceof String) {
                    PathPlannerPath path = PathPlannerPath.fromPathFile((String) item);

                    if (pathFinded == false) {
                        // needed Values
                        Pose2d pathStartPose = path.getStartingDifferentialPose();
                        Rotation2d targetRot = pathStartPose.getRotation();
                        BooleanSupplier conditional = () -> {
                            Rotation2d currentRot = m_poseEstimator.getEstimatedPosition().getRotation();
                            double diff = Math.abs((targetRot.minus(currentRot)).getDegrees());

                            return diff <= 45;
                        };

                        // pathfind command
                        Command pathfinderCommand = AutoBuilder.pathfindToPose(
                            pathStartPose, 
                            m_constraints, 
                            path.getIdealStartingState().velocityMPS()
                        );

                        // path and rotate stuff
                        Command rotateCommand = new RotateToRotation2D(
                            m_DriveSubsystem, 
                            m_poseEstimator,
                            targetRot,
                            2.0
                        );

                        Command testCommand = new InstantCommand(() -> {
                            Pose2d currentPose = m_poseEstimator.getEstimatedPosition();

                            List<Pose2d> pathPoses = path.getPathPoses();
                            pathPoses.set(0, currentPose);

                            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(pathPoses);
                            PathPlannerPath newPath = new PathPlannerPath(waypoints, path.getGlobalConstraints(), path.getIdealStartingState(), path.getGoalEndState());
                            Command newPathCommand = AutoBuilder.followPath(newPath).andThen(commandGroup);

                            System.out.println(" -- scheduling everyhting else --");
                            System.out.println(" -- scheduling everyhting else --");
                            System.out.println(" -- scheduling everyhting else --");
                            System.out.println(" -- scheduling everyhting else --");
                            CommandScheduler.getInstance().schedule(newPathCommand);
                        });

                        // decide what to do
                        Command conditonalCommand = Commands.either(
                            new InstantCommand(), 
                            rotateCommand, 
                            conditional
                        );

                        // add to group
                        precommandGroup.addCommands(pathfinderCommand, conditonalCommand, testCommand);

                        // Command pathfinderCommand = AutoBuilder.pathfindThenFollowPath(path, m_constraints);
                        // commandGroup.addCommands(pathfinderCommand);

                        pathFinded = true;
                    } else {
                        Command pathCommand = AutoBuilder.followPath(path);
                        commandGroup.addCommands(pathCommand);
                    }
                }
            }

            CommandScheduler.getInstance().schedule(precommandGroup);
        } catch (Exception e) {
            // do somthing here
            System.out.println("Failed to run " + this.getName() + ": " + e.toString());
        }
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
