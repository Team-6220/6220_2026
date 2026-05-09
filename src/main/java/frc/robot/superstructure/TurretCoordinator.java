package frc.robot.superstructure;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.turret.TurretSubsystem;
import java.util.Arrays;

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
  // Optional visualization objects (part of Swerve's Field2d)
  private FieldObject2d turretObject = null;
  private FieldObject2d turretAimObject = null;

  // Current mode
  private AimMode mode = AimMode.AIM_AT_HUB;

  // Field-relative angle for the "other side"
  private Rotation2d fixedFieldAngle = Rotation2d.fromDegrees(0);

  // NOTE: Previously used a midline threshold. This coordinator now uses the alliance hub X
  // coordinate as the threshold; MIDLINE is retained in history but not used.

  /** Basic coordinator without visualization. */
  public TurretCoordinator(TurretSubsystem turret) {
    this.turret = turret;
  }

  /**
   * Coordinator that updates visualization objects on the provided Field2d. Pass in the
   * Swerve.getField2d() instance so the coordinator will update turret pose and aim each loop.
   */
  public TurretCoordinator(TurretSubsystem turret, Field2d field) {
    this.turret = turret;
    try {
      this.turretObject = field.getObject("turret");
      this.turretAimObject = field.getObject("turretAim");
    } catch (Exception e) {
      // If field is null or unavailable, keep visualization disabled
      this.turretObject = null;
      this.turretAimObject = null;
    }
  }

  /** Called every loop by a default command */
  public void update(Pose2d robotPose) {
    updateMode(robotPose);
    // Compute desired angle in robot-relative coordinates (what the turret controller expects)
    Rotation2d desiredRobotRel = computeDesiredRobotRelativeAngle(robotPose);
    turret.driveToGoal(desiredRobotRel);

    // Update visualization if available: compute turret world pose (robot pose + turret offset)
    if (turretObject != null) {
      // Rotate turret offset by robot yaw then add to robot translation
      Rotation2d robotYaw = robotPose.getRotation();
      Translation2d offset = FieldConstants.TURRET_OFFSET_XY.rotateBy(robotYaw);
      Translation2d turretTrans = robotPose.getTranslation().plus(offset);

      // desiredRobotRel is robot-relative; convert to world rotation for visualization
      Rotation2d turretWorldRot = robotYaw.rotateBy(desiredRobotRel);
      Pose2d turretPose = new Pose2d(turretTrans, turretWorldRot);
      try {
        turretObject.setPose(turretPose);
      } catch (Exception e) {
        // ignore visualization errors
      }

      // Also draw an aim line (two poses: turret center -> point along aim direction)
      if (turretAimObject != null) {
        double len = Meters.of(1.0).in(Meters); // 1 meter long aim line
        double dx = Math.cos(turretWorldRot.getRadians()) * len;
        double dy = Math.sin(turretWorldRot.getRadians()) * len;
        Pose2d end = new Pose2d(turretTrans.plus(new Translation2d(dx, dy)), turretWorldRot);
        try {
          turretAimObject.setPoses(Arrays.asList(turretPose, end));
        } catch (Exception e) {
          // ignore
        }
      }
    }
  }

  /** Automatically switch modes based on robot location */
  private void updateMode(Pose2d robotPose) {
    // Use the X coordinate of the alliance hub as the threshold. When the robot crosses the
    // hub's X (for the current alliance) we switch to AIM_AT_HUB mode. This makes the mode
    // change track the team-specific hub location instead of the field midline.
    double hubX;
    if (Constants.isRed != null && Constants.isRed.equals("red")) {
      hubX = FieldConstants.RED_HUB_POSE.getX();
      mode = robotPose.getX() > hubX ? AimMode.AIM_AT_HUB : AimMode.FIELD_RELATIVE_ANGLE;
    } else {
      hubX = FieldConstants.BLUE_HUB_POSE.getX();
      mode = robotPose.getX() < hubX ? AimMode.AIM_AT_HUB : AimMode.FIELD_RELATIVE_ANGLE;
    }
  }

  /** Compute the turret angle based on the current mode */
  /**
   * Returns the desired turret angle in field-relative coordinates (Rotation2d expressed in the
   * field/world frame).
   */
  private Rotation2d computeDesiredFieldRelativeAngle(Pose2d robotPose) {
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

  /**
   * Returns the desired turret angle in robot-relative coordinates. This converts the
   * field-relative desired angle into the turret's robot frame so it can be passed to the turret
   * drive command (which expects a Rotation2d relative to the robot forward).
   */
  private Rotation2d computeDesiredRobotRelativeAngle(Pose2d robotPose) {
    // Field-relative desired angle
    Rotation2d desiredField = computeDesiredFieldRelativeAngle(robotPose);
    // Convert to robot frame by subtracting robot yaw: angle_robot = angle_field - robot_yaw
    double angleField = desiredField.getRadians();
    double robotYaw = robotPose.getRotation().getRadians();
    double angleRobot = angleField - robotYaw;
    return new Rotation2d(angleRobot);
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
