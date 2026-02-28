package frc.robot.commands.util;

import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.RotateToRotation2D;
import frc.robot.subsystems.DriveSubsystem;

public class PathfindThenAutoCommand extends Command {
    private final DriveSubsystem m_driveSubsystem;
    private final DifferentialDrivePoseEstimator m_poseEstimator;
    private final PathConstraints m_constraints;
    private final String m_plannedAutoName;
    private final BooleanSupplier m_isRed;

    public PathfindThenAutoCommand(DriveSubsystem driveSubsystem, DifferentialDrivePoseEstimator poseEstimator, PathConstraints constraints, String plannedAutoName, BooleanSupplier isRed) 
    {
        m_driveSubsystem = driveSubsystem;
        m_poseEstimator = poseEstimator;
        m_constraints = constraints;

        m_plannedAutoName = plannedAutoName;
        m_isRed = isRed;
    }

    @Override
    public void initialize() {
        // PathPlannerAuto plannedAuto = new PathPlannerAuto(m_plannedAutoName);
        // Pose2d startingPose = plannedAuto.getStartingPose();

        // if (m_isRed.getAsBoolean() == true) {
        //     startingPose = FlippingUtil.flipFieldPose(startingPose);
        // }

        // Command rotateToStartRot = new RotateToRotation2D(
        //     m_driveSubsystem, 
        //     m_poseEstimator,
        //     startingPose.getRotation(), 
        //     3.0
        // );

        // Command pathfindToStartPose = AutoBuilder.pathfindToPose(startingPose, m_constraints);
        // Command pathfindThenAuto = Commands.sequence(pathfindToStartPose, rotateToStartRot, plannedAuto);

        // CommandScheduler.getInstance().schedule(pathfindThenAuto);

        // TODO: Look into named commands and finish the command rewrite.
        try {
            List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(m_plannedAutoName);

            // check if empty, if it isnt then remove first value
            // take the first value and plug it into a Pathfindthenfollowpath
            // then create a sequenatal command group and add the pathfind to it
            // then loop through the rest of the paths and add it to the command group.
            
            // also check if named commands trasnfer, if not need to figure that out. (trigger?)
        } catch (Exception e) {
            // do somthing here
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
