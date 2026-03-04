package frc.robot.utils;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;

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
}
