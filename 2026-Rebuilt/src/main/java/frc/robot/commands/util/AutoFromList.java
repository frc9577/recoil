package frc.robot.commands.util;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.ConditionalRotation;
import frc.robot.utils.PathUtils;

// Auto From Path Names And Commands
public class AutoFromList extends Command {
    private final List<Object> m_sequence;
    private final PathConstraints m_constraints;
    private final DriveSubsystem m_DriveSubsystem;
    private final DifferentialDrivePoseEstimator m_poseEstimator;
    private final firstPathType m_pathType;

    public enum firstPathType {
        NONE,
        GO_TO_FIRST_PATH,
        REPLACE_FIRST_PATH,
    }

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
        m_pathType = firstPathType.GO_TO_FIRST_PATH;

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
     * @param goToFirstPath Should the command pathfind to the first path?
     */
    public AutoFromList(List<Object> sequence, PathConstraints constraints, DriveSubsystem driveSubsystem, DifferentialDrivePoseEstimator poseEstimator, firstPathType pathType) 
    {
        m_sequence = sequence;
        m_constraints = constraints;
        m_DriveSubsystem = driveSubsystem;
        m_poseEstimator = poseEstimator;
        m_pathType = pathType;

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
     * @param goToFirstPath Should the command go to the first path?
     * @param doWarmup Should the command warm-up the paths itself? (If instantiated and ran during run-time, you might want this to be set to false)
     */
    public AutoFromList(List<Object> sequence, PathConstraints constraints, DriveSubsystem driveSubsystem, DifferentialDrivePoseEstimator poseEstimator, firstPathType pathType, boolean doWarmup) 
    {
        m_sequence = sequence;
        m_constraints = constraints;
        m_DriveSubsystem = driveSubsystem;
        m_poseEstimator = poseEstimator;
        m_pathType = pathType;

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
            Boolean pathFinded = m_pathType == firstPathType.NONE; // If we want to pathfind this will be set to false.

            for (Object item : m_sequence) {
                if (item instanceof Command) {
                    if (pathFinded == false) {
                        precommandGroup.addCommands((Command) item);
                        System.out.println("Added " + item.getClass().getName() + " to preCommandGroup!");
                    } else {
                        commandGroup.addCommands((Command) item);
                        System.out.println("Added " + item.getClass().getName() + " to commandGroup!");
                    }
                } else if (item instanceof String) {
                    PathPlannerPath path = PathPlannerPath.fromPathFile((String) item);

                    if (pathFinded == false) {
                        Command createEverythingCommand = Commands.runOnce(() -> {
                            // needed Values
                            Pose2d pathStartPose = path.getStartingDifferentialPose();
                            Command rotateTwordPathStart = ConditionalRotation.New(m_DriveSubsystem, m_poseEstimator, pathStartPose.getRotation(), 45, 2.0);

                            // path edits
                            Command pathOnTheFly = Commands.runOnce(() -> {
                                Pose2d currentPose = m_poseEstimator.getEstimatedPosition();

                                PathPlannerPath newPath; 
                                if (m_pathType == firstPathType.GO_TO_FIRST_PATH) {
                                    newPath = PathUtils.appendToPath(path, 0, currentPose);
                                } else if (m_pathType == firstPathType.REPLACE_FIRST_PATH) {
                                    newPath = PathUtils.modifyPath(path, new Pair<Integer, Pose2d>(0, currentPose));
                                } else {
                                    newPath = path;
                                }

                                Command newPathCommand = AutoBuilder.followPath(newPath);

                                // schedule the post first-path stuff
                                System.out.println("Schedueling commandGroup!");
                                CommandScheduler.getInstance().schedule(
                                    newPathCommand
                                    .andThen(commandGroup)
                                );
                            });

                            // schedule the first pre-path stuff
                            System.out.println("Schedueling the path on the fly stuff!");
                            CommandScheduler.getInstance().schedule(
                                rotateTwordPathStart
                                .andThen(pathOnTheFly)
                            );
                        });
                        
                        pathFinded = true;
                        precommandGroup.addCommands(createEverythingCommand);
                        System.out.println("Added createEverything to preCommandGroup!");
                    } else {
                        Command pathCommand = AutoBuilder.followPath(path);
                        commandGroup.addCommands(pathCommand);
                    }
                }
            }

            System.out.println("Schedueling preCommandGroup!");
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
