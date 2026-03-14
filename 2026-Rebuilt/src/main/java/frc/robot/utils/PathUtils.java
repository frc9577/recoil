package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class PathUtils {
    public static List<Waypoint> deepCopyWaypoints(List<Waypoint> base) {
        List<Waypoint> newWaypoints = new ArrayList<>();
        for (Waypoint w : base) {
            Translation2d prev_o = w.prevControl();
            Translation2d anchor_o = w.anchor();
            Translation2d next_o = w.nextControl();

            Translation2d prev = null;
            if (prev_o != null) {
                prev = new Translation2d(prev_o.getX(), prev_o.getY());
            }

            Translation2d anchor = null;
            if (anchor_o != null) {
                anchor = new Translation2d(anchor_o.getX(), anchor_o.getY());
            }

            Translation2d next = null;
            if (next_o != null) {
                next = new Translation2d(next_o.getX(), next_o.getY());
            }

            newWaypoints.add(new Waypoint(prev, anchor, next));            
        }

        return newWaypoints;
    }

    /**
     * Allows you to do the modification of a path w/o needing to do the jankyness yourself.
     * 
     * @param basePath The path that is used as a base.
     * @param pointsToModify A pair that represents the index of a point on the path, and the replacement value.
     * @return The path with the modified positions.
    */
    @SafeVarargs
    public static PathPlannerPath modifyPath(PathPlannerPath basePath, Pair<Integer, Pose2d>... pointsToModify) {
        List<Waypoint> pathWaypoints = deepCopyWaypoints(basePath.getWaypoints());
        for (Pair<Integer, Pose2d> point : pointsToModify) {
            Integer index = point.getFirst();
            if (pathWaypoints.size() > index && index >= 0) {
                Waypoint oldPoint = pathWaypoints.get(index);
                Waypoint newPoint = new Waypoint(
                    oldPoint.prevControl(), 
                    point.getSecond().getTranslation(), 
                    oldPoint.nextControl()
                );

                pathWaypoints.set(index, newPoint);
            }
        }

        PathPlannerPath newPath = new PathPlannerPath(
            pathWaypoints, 
            basePath.getGlobalConstraints(), 
            basePath.getIdealStartingState(), 
            basePath.getGoalEndState(), 
            basePath.isReversed()
        );

        return newPath;
    }

    /**
     * Append a set of points to the base path.
     * 
     * @param basePath The path that is used as a base.
     * @param startIndex The index of where to append to, 0 is the start and -1 is the end.
     * @param pointsToAppend The poses to append at the location, in order.
     * @return The path with the modified positions.
    */
    public static PathPlannerPath appendToPath(PathPlannerPath basePath, Integer index, Pose2d pointToAppend) {
        List<Waypoint> pathWaypoints = deepCopyWaypoints(basePath.getWaypoints());

        Translation2d newAnchor = pointToAppend.getTranslation();
        Translation2d prevControl = null;
        Translation2d nextControl = null; 

        int lastWaypointIndex = pathWaypoints.size()-1;
        if (index == -1) {
            index = lastWaypointIndex;
        } 
        
        if (lastWaypointIndex == index) {
            Waypoint prev = pathWaypoints.get(index);
            prevControl = prev.anchor();
            pathWaypoints.set(index, new Waypoint(prev.prevControl(), prev.anchor(), newAnchor));
        } else if (index == 0) {
            Waypoint next = pathWaypoints.get(index);
            nextControl = next.anchor();
            pathWaypoints.set(index, new Waypoint(newAnchor, next.anchor(), next.nextControl()));
        } else if (index > 0 && index < lastWaypointIndex) {
            Waypoint prev = pathWaypoints.get(index-1);
            prevControl = prev.anchor();
            pathWaypoints.set(index-1, new Waypoint(prev.prevControl(), prev.anchor(), newAnchor));

            Waypoint next = pathWaypoints.get(index);
            nextControl = next.anchor();
            pathWaypoints.set(index, new Waypoint(newAnchor, next.anchor(), next.nextControl()));
        }

        if (pathWaypoints.size() > index) {
            Waypoint point = new Waypoint(prevControl, newAnchor, nextControl);
            pathWaypoints.add(index, point);
        }

        PathPlannerPath newPath = new PathPlannerPath(
            pathWaypoints, 
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
