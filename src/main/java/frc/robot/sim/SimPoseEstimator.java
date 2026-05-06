package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Very small simulated pose estimator (ground-truth pass-through) that mimics the
 * SwervePoseEstimator API used by the robot. This returns the input pose as the estimated pose and
 * supports addVisionMeasurement to emulate vision resets.
 */
public class SimPoseEstimator {
  private Pose2d pose = new Pose2d(0, 0, new Rotation2d());

  public synchronized void resetPosition(Pose2d p) {
    pose = p;
  }

  public synchronized Pose2d getEstimatedPosition() {
    return pose;
  }

  public synchronized void update(Pose2d p) {
    pose = p;
  }

  public synchronized void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
    // Simple behavior: immediately trust vision measurement (for sim purposes)
    pose = visionPose;
  }
}
