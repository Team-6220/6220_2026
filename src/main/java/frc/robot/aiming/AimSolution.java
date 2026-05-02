package frc.robot.aiming;

import edu.wpi.first.math.geometry.Pose2d;

public class AimSolution {
  public final Pose2d interceptPose;
  public final double timeOfFlight;
  public final double requiredTurretAngleRad;
  public final double confidence; // 0..1

  public AimSolution(Pose2d p, double tof, double angleRad, double conf) {
    interceptPose = p;
    timeOfFlight = tof;
    requiredTurretAngleRad = angleRad;
    confidence = conf;
  }
}
