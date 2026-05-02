package frc.robot.aiming;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.ballistics.ProjectileModel;

/**
 * Simple aiming predictor: predicts intercept position assuming target moves at constant velocity
 * and projectile speed is constant horizontally.
 */
public final class AimPredictor {
  private AimPredictor() {}

  public static AimSolution predictIntercept(
      Pose2d targetPose, Translation2d targetVel, Pose2d shooterPose, double projectileSpeed) {
    double range = shooterPose.getTranslation().getDistance(targetPose.getTranslation());
    double tof = ProjectileModel.estimateTimeOfFlight(range, projectileSpeed);

    // Predict future target position
    double futureX = targetPose.getX() + targetVel.getX() * tof;
    double futureY = targetPose.getY() + targetVel.getY() * tof;
    Pose2d intercept = new Pose2d(new Translation2d(futureX, futureY), new Rotation2d());

    // Turret angle in robot-frame: compute vector from shooter to intercept
    double dx = futureX - shooterPose.getX();
    double dy = futureY - shooterPose.getY();
    double angle = Math.atan2(dy, dx);

    double confidence = Double.isFinite(tof) ? 1.0 : 0.0;

    return new AimSolution(intercept, tof, angle, confidence);
  }
}
