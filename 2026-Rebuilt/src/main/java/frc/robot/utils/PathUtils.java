package frc.robot.utils;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PathUtils {
    /**
     * Allows you to do the modification of a path w/o needing to do the jankyness yourself.
     * 
     * @param basePath The path that is used as a base.
     * @param pointsToModify A pair that represents the index of a point on the path, and the replacement value.
     * @return The path with the modified positions.
    */
    @SafeVarargs
    public static PathPlannerPath modifyPath(PathPlannerPath basePath, Pair<Integer, Pose2d>... pointsToModify) {
        List<Pose2d> pathPoses = basePath.getPathPoses();
        for (Pair<Integer, Pose2d> point : pointsToModify) {
            Integer index = point.getFirst();
            if (pathPoses.size() > index && index >= 0) {
                pathPoses.set(index, point.getSecond());
            }
        }

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(pathPoses);
        PathPlannerPath newPath = new PathPlannerPath(
            waypoints, 
            basePath.getGlobalConstraints(), 
            basePath.getIdealStartingState(), 
            basePath.getGoalEndState(), 
            basePath.isReversed()
        );

        return newPath;
    }

    /**
    * This creates a path on the fly to a given target point.
    *
    * @param startPose The pose the robot will be at the start of the path.
    * @param targetPose The pose the robot will make a point to go to.
    * @param constraints The constraints the robot will follow while following the path.
    * @param endVelocity The velocity it should be at the end of the path.
    * @param reversed If the robot should run the path in reversed (backward)
    */
    public static PathPlannerPath OnTheFlyToTarget(Pose2d startPose, Pose2d targetPose, PathConstraints constraints, Double endVelocity, Boolean reversed) 
    {
        // Create a list of waypoints from poses. Each pose represents one waypoint.
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        Rotation2d rotateMod = Rotation2d.kZero;
        if (reversed) {
            rotateMod = Rotation2d.k180deg;
        }

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(startPose.getTranslation(), startPose.getRotation().minus(rotateMod)),
            new Pose2d(targetPose.getTranslation(), targetPose.getRotation().minus(rotateMod))
        );

        // Create the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            new IdealStartingState(0.0, startPose.getRotation()), // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState(endVelocity, targetPose.getRotation()),
            reversed
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;
        
        // Create the command version of the path and return it
        return path;
    }

    /**
     * This creates a path on the fly to a given target point.
     *
     * @param startPose The pose the robot will be at the start of the path.
     * @param targetPose The pose the robot will make a point to go to.
     * @param constraints The constraints the robot will follow while following the path.
     * @param endVelocity The velocity it should be at the end of the path.
     */
    public static PathPlannerPath OnTheFlyToTarget(Pose2d startPose, Pose2d targetPose, PathConstraints constraints, Double endVelocity) 
    {
        return OnTheFlyToTarget(startPose, targetPose, constraints, endVelocity, false);
    }

    /**
     * This creates a path on the fly to a given target point.
     *
     * @param startPose The pose the robot will be at the start of the path.
     * @param targetPose The pose the robot will make a point to go to.
     * @param constraints The constraints the robot will follow while following the path.
     * @param reversed If the robot should run the path in reversed (backward)
     */
    public static PathPlannerPath OnTheFlyToTarget(Pose2d startPose, Pose2d targetPose, PathConstraints constraints, Boolean reversed) 
    {
        return OnTheFlyToTarget(startPose, targetPose, constraints, 0.0, reversed);
    }

    /**
     * This creates a path on the fly to a given target point.
     * 
     * @param startPose The pose the robot will be at the start of the path.
     * @param targetPose The pose the robot will make a point to go to.
     * @param constraints The constraints the robot will follow while following the path.
     */
    public static PathPlannerPath OnTheFlyToTarget(Pose2d startPose, Pose2d targetPose, PathConstraints constraints) 
    {
        return OnTheFlyToTarget(startPose, targetPose, constraints, 0.0, false);
    }
}
