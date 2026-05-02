package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.aiming.AimPredictor;
import frc.robot.aiming.AimSolution;
import frc.robot.turret.TurretSubsystem;

/** Small coordinator that requests aim solutions and commands the turret. */
public class SuperstructureCoordinator {
  private final TurretSubsystem turret;

  public SuperstructureCoordinator(TurretSubsystem turret) {
    this.turret = turret;
  }

  public void aimAndFire(
      Pose2d targetPose, Translation2d targetVel, Pose2d shooterPose, double projectileSpeed) {
    AimSolution sol =
        AimPredictor.predictIntercept(targetPose, targetVel, shooterPose, projectileSpeed);
    if (sol.confidence > 0.5) {
      turret.setTargetAngle(new edu.wpi.first.math.geometry.Rotation2d(sol.requiredTurretAngleRad));
      // In real code, check turret.atSetpoint(), shooter RPM, hood angle, then fire
    }
  }
}
