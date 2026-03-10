package frc.robot.commands.util;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.ConditionalRotation;
import frc.robot.utils.PathUtils;
import frc.robot.utils.PoseDiff;
import edu.wpi.first.math.Pair;

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
     * By default this command pathfinds to the first path and warm-ups the paths itself.
     * 
     * @param sequence A list of path names (strings) and commands to run in order.
     * @param constraints The constraints the pathfinder segment of the auto uses.
     * @param driveSubsystem The driveSubsystem, pass-ins for rotation corrections.
     * @param poseEstimator The pose estimator, used to get the robots location.
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

    /**
     * Creates and runs a new auto from a list of path names and commands.
     * Any list with a command can only be ran once due to how the scheudler works.
     * By default this command warm-ups the paths itself.
     * 
     * @param sequence A list of path names (strings) and commands to run in order.
     * @param constraints The constraints the pathfinder segment of the auto uses.
     * @param driveSubsystem The driveSubsystem, pass-ins for rotation corrections.
     * @param poseEstimator The pose estimator, used to get the robots location.
     * @param pathfindToFirstpath Should the command pathfind to the first path?
     */
    public AutoFromList(List<Object> sequence, PathConstraints constraints, DriveSubsystem driveSubsystem, DifferentialDrivePoseEstimator poseEstimator, boolean pathfindToFirstpath) 
    {
        m_sequence = sequence;
        m_constraints = constraints;
        m_DriveSubsystem = driveSubsystem;
        m_poseEstimator = poseEstimator;
        m_pathfindToFirstPath = pathfindToFirstpath;

        warmup();
    }

    /**
     * Creates and runs a new auto from a list of path names and commands.
     * Any list with a command can only be ran once due to how the scheudler works.
     * 
     * @param sequence A list of path names (strings) and commands to run in order.
     * @param constraints The constraints the pathfinder segment of the auto uses.
     * @param driveSubsystem The driveSubsystem, pass-ins for rotation corrections.
     * @param poseEstimator The pose estimator, used to get the robots location.
     * @param pathfindToFirstpath Should the command pathfind to the first path?
     * @param doWarmup Should the command warm-up the paths itself? (If instantiated and ran during run-time, you might want this to be set to false)
     */
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
                        Command createEverythingCommand = new InstantCommand(() -> {
                            // needed Values
                            Pose2d pathStartPose = path.getStartingDifferentialPose(); // Accounts for rotation

                            Rotation2d directionRot = pathStartPose.getRotation();
                            System.out.println("Direction ROT: " + String.valueOf(directionRot.getDegrees()));

                            PoseDiff dPose = new PoseDiff(m_poseEstimator.getEstimatedPosition(), pathStartPose);
                            Rotation2d rotToPoint = new Rotation2d(Math.atan2(dPose.y, dPose.x));

                            // pathfind command
                            Command pathfinderCommand = AutoBuilder.pathfindToPose(
                                pathStartPose, 
                                m_constraints, 
                                path.getIdealStartingState().velocityMPS()
                            );

                            Command postPathfindCommand = new InstantCommand(() -> {
                                Pose2d currentPose = m_poseEstimator.getEstimatedPosition();

                                PathPlannerPath newPath = PathUtils.modifyPath(path, new Pair<>(0, currentPose));
                                Command newPathCommand = AutoBuilder.followPath(newPath).andThen(commandGroup);

                                CommandScheduler.getInstance().schedule(newPathCommand);
                            });

                            // decide what to do
                            Command rotateTwordPathConditional = ConditionalRotation.New(m_DriveSubsystem, m_poseEstimator, rotToPoint, 90, 2.0);
                            Command rotateTwordStartRotationConditional = ConditionalRotation.New(m_DriveSubsystem, m_poseEstimator, directionRot, 45, 2.0);

                            // schedule it
                            CommandScheduler.getInstance().schedule(
                                rotateTwordStartRotationConditional
                                .andThen(pathfinderCommand)
                                .andThen(rotateTwordPathConditional)
                                .andThen(postPathfindCommand)
                            );
                        });
                        
                        pathFinded = true;
                        precommandGroup.addCommands(createEverythingCommand);
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
