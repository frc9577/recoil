package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;

public class PoseDiff {
    public final double x;
    public final double y;

    // pose2 - pose 1
    public PoseDiff(Pose2d pose1, Pose2d pose2) {
        this.x = pose2.getX() - pose1.getX();
        this.y = pose2.getY() - pose1.getY();
    }
}