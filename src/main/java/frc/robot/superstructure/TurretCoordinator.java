package frc.robot.superstructure;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.turret.TurretSubsystem;

/**
 * TurretCoordinator: High-level brain that decides what the turret should aim at. - AIM_AT_HUB on
 * one side of the field - FIELD_RELATIVE_ANGLE on the other side
 *
 * <p>Subsystem stays dumb (PID to angle). Commands simply call update() every loop.
 */
public class TurretCoordinator {

  /** Aiming modes */
  public enum AimMode {
    AIM_AT_HUB,
    FIELD_RELATIVE_ANGLE
  }

  private final TurretSubsystem turret;

  // Current mode
  private AimMode mode = AimMode.AIM_AT_HUB;

  // Field-relative angle for the "other side"
  private Rotation2d fixedFieldAngle = Rotation2d.fromDegrees(180);

  // Midline X coordinate (example — adjust to your field)
  private static final double MIDLINE_X = FieldConstants.FIELD_LENGTH.in(Meters) / 2.0;

  public TurretCoordinator(TurretSubsystem turret) {
    this.turret = turret;
  }

  /** Called every loop by a default command */
  public void update(Pose2d robotPose) {
    updateMode(robotPose);
    Rotation2d desired = computeDesiredAngle(robotPose);
    turret.driveToGoal(desired);
  }

  /** Automatically switch modes based on robot location */
  private void updateMode(Pose2d robotPose) {
    // Use Constants.isRed to determine alliance-side behavior. If isRed == "red" we treat the
    // right side (X > midline) as the hub side; otherwise (blue/unknown) we treat the left side
    // (X < midline) as the hub side.
    if (Constants.isRed != null && Constants.isRed.equals("red")) {
      mode = robotPose.getX() > MIDLINE_X ? AimMode.AIM_AT_HUB : AimMode.FIELD_RELATIVE_ANGLE;
    } else {
      mode = robotPose.getX() < MIDLINE_X ? AimMode.AIM_AT_HUB : AimMode.FIELD_RELATIVE_ANGLE;
    }
  }

  /** Compute the turret angle based on the current mode */
  private Rotation2d computeDesiredAngle(Pose2d robotPose) {
    switch (mode) {
      case AIM_AT_HUB:
        return computeHubAngle(robotPose);

      case FIELD_RELATIVE_ANGLE:
        // Flip the fixed field angle based on Constants.isRed so "other-side" angle is correct.
        if (Constants.isRed != null && !Constants.isRed.equals("red")) {
          return fixedFieldAngle.rotateBy(Rotation2d.fromDegrees(180));
        }
        return fixedFieldAngle;

      default:
        return new Rotation2d();
    }
  }

  /** Compute angle from robot → HUB */
  private Rotation2d computeHubAngle(Pose2d robotPose) {
    Translation2d hub =
        Constants.isRed.equals("red")
            ? FieldConstants.RED_HUB_POSE.getTranslation()
            : FieldConstants.BLUE_HUB_POSE.getTranslation();
    Translation2d diff = hub.minus(robotPose.getTranslation());
    double angleRad = Math.atan2(diff.getY(), diff.getX());
    return new Rotation2d(angleRad);
  }

  /** Optional: allow operator to override the fixed angle */
  public void setFixedFieldAngle(Rotation2d angle) {
    this.fixedFieldAngle = angle;
  }

  /** Optional: allow operator to force a mode */
  public void setMode(AimMode newMode) {
    this.mode = newMode;
  }

  public AimMode getMode() {
    return mode;
  }
}
