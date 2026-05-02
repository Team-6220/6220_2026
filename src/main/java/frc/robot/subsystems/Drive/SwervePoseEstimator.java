package frc.robot.subsystems.Drive;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/**
 * Thin wrapper around WPILib's SwerveDrivePoseEstimator to centralize pose-related helpers and
 * provide a stable, testable API for the robot code.
 */
public class SwervePoseEstimator {
  private final SwerveDrivePoseEstimator estimator;

  public SwervePoseEstimator(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions,
      Pose2d initialPose,
      Vector stateStdDevs,
      Vector visionStdDevs) {
    estimator =
        new SwerveDrivePoseEstimator(
            kinematics, gyroAngle, modulePositions, initialPose, stateStdDevs, visionStdDevs);
  }

  public Pose2d getEstimatedPosition() {
    return estimator.getEstimatedPosition();
  }

  public void update(Rotation2d gyroYaw, SwerveModulePosition[] modulePositions) {
    estimator.update(gyroYaw, modulePositions);
  }

  public void resetPosition(
      Rotation2d gyroYaw, SwerveModulePosition[] modulePositions, Pose2d pose) {
    estimator.resetPosition(gyroYaw, modulePositions, pose);
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
    estimator.addVisionMeasurement(visionPose, timestampSeconds);
  }
}
