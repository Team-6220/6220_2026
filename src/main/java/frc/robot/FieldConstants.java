package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;

/**
 * Simulation geometry constants to align the sim with the real robot and field.
 *
 * <p>Tune these values to match the actual robot geometry and field layout.
 */
public final class FieldConstants {
  private FieldConstants() {}

  // Field geometry (meters) from 2026 Game Manual section 5.2
  public static final Distance FIELD_LENGTH = Meters.of(16.54); // 651.2 in
  public static final Distance FIELD_WIDTH = Meters.of(8.07); // 317.7 in

  // Hub geometry from 2026 Game Manual section 5.4
  public static final Distance HUB_SIZE = Meters.of(1.19); // 47 in
  public static final Distance HUB_TOP_OPENING_HEIGHT =
      Meters.of(1.83); // 72 in (front edge of opening)

  // Hub positions in field coordinates (meters)
  public static final Pose2d BLUE_HUB_POSE =
      new Pose2d(Meters.of(4.625594), Meters.of(4.034536), new Rotation2d());
  public static final Pose2d RED_HUB_POSE =
      new Pose2d(Meters.of(11.915394), Meters.of(4.034536), new Rotation2d());

  // Hub height (meters) for 3D visualization (use opening height)
  public static final Distance HUB_HEIGHT = HUB_TOP_OPENING_HEIGHT;

  // Turret geometry relative to robot center
  public static final Translation2d TURRET_OFFSET_XY =
      new Translation2d(Meters.of(0.0), Meters.of(0.0)); // meters
  public static final Distance TURRET_HEIGHT = Meters.of(0.5); // meters above floor

  // Camera geometry
  // Check "WPILIB Coordinate" and MAKE SURE YOU HAVE THIS RIGHT
  public static final Transform3d CAMERA_TO_ROBOT =
      new Transform3d(
          new Translation3d(Meters.of(0.2), Meters.of(0.0), Meters.of(0.7)), new Rotation3d());

  // If you have multiple cameras, create additional transforms here.

  // AprilTag poses for the 2026 field
  public static final Pose3d[] APRILTAG_POSES = {
    new Pose3d(
        Meters.of(-3.604959),
        Meters.of(-3.3899914),
        Meters.of(0.889),
        new Rotation3d(0, 0, Math.PI)),
    new Pose3d(
        Meters.of(-3.6423986),
        Meters.of(-0.6032558),
        Meters.of(1.12395),
        new Rotation3d(0, 0, Math.PI / 2)),
    new Pose3d(
        Meters.of(-3.0388438),
        Meters.of(-0.3554534),
        Meters.of(1.12395),
        new Rotation3d(0, 0, Math.PI)),
    new Pose3d(
        Meters.of(-3.0388438),
        Meters.of(0.0001466),
        Meters.of(1.12395),
        new Rotation3d(0, 0, Math.PI)),
    new Pose3d(
        Meters.of(-3.6423986),
        Meters.of(0.603549),
        Meters.of(1.12395),
        new Rotation3d(0, 0, -Math.PI / 2)),
    new Pose3d(
        Meters.of(-3.604959),
        Meters.of(3.3902846),
        Meters.of(0.889),
        new Rotation3d(0, 0, Math.PI)),
    new Pose3d(
        Meters.of(-3.6798636), Meters.of(3.3902846), Meters.of(0.889), new Rotation3d(0, 0, 0)),
    new Pose3d(
        Meters.of(-3.9979986),
        Meters.of(0.603549),
        Meters.of(1.12395),
        new Rotation3d(0, 0, -Math.PI / 2)),
    new Pose3d(
        Meters.of(-4.2461566), Meters.of(0.3557466), Meters.of(1.12395), new Rotation3d(0, 0, 0)),
    new Pose3d(
        Meters.of(-4.2461566), Meters.of(0.0001466), Meters.of(1.12395), new Rotation3d(0, 0, 0)),
    new Pose3d(
        Meters.of(-3.9979986),
        Meters.of(-0.6032558),
        Meters.of(1.12395),
        new Rotation3d(0, 0, Math.PI / 2)),
    new Pose3d(
        Meters.of(-3.6798636), Meters.of(-3.3899914), Meters.of(0.889), new Rotation3d(0, 0, 0)),
    new Pose3d(
        Meters.of(-8.240332),
        Meters.of(-3.370408),
        Meters.of(0.55245),
        new Rotation3d(0, 0, Math.PI)),
    new Pose3d(
        Meters.of(-8.240332),
        Meters.of(-2.938608),
        Meters.of(0.55245),
        new Rotation3d(0, 0, Math.PI)),
    new Pose3d(
        Meters.of(-8.2399764),
        Meters.of(-0.2909882),
        Meters.of(0.55245),
        new Rotation3d(0, 0, Math.PI)),
    new Pose3d(
        Meters.of(-8.2399764),
        Meters.of(0.1408118),
        Meters.of(0.55245),
        new Rotation3d(0, 0, Math.PI)),
    new Pose3d(
        Meters.of(3.6099364), Meters.of(3.3902846), Meters.of(0.889), new Rotation3d(0, 0, 0)),
    new Pose3d(
        Meters.of(3.6474014),
        Meters.of(0.603549),
        Meters.of(1.12395),
        new Rotation3d(0, 0, Math.PI / 2)),
    new Pose3d(
        Meters.of(3.0438466), Meters.of(0.3557466), Meters.of(1.12395), new Rotation3d(0, 0, 0)),
    new Pose3d(
        Meters.of(3.0438466), Meters.of(0.0001466), Meters.of(1.12395), new Rotation3d(0, 0, 0)),
    new Pose3d(
        Meters.of(3.6474014),
        Meters.of(-0.6032558),
        Meters.of(1.12395),
        new Rotation3d(0, 0, -Math.PI / 2)),
    new Pose3d(
        Meters.of(3.6099364), Meters.of(-3.3899914), Meters.of(0.889), new Rotation3d(0, 0, 0)),
    new Pose3d(
        Meters.of(3.684841),
        Meters.of(-3.3899914),
        Meters.of(0.889),
        new Rotation3d(0, 0, Math.PI)),
    new Pose3d(
        Meters.of(4.0030014),
        Meters.of(-0.6032558),
        Meters.of(1.12395),
        new Rotation3d(0, 0, Math.PI / 2)),
    new Pose3d(
        Meters.of(4.251134),
        Meters.of(-0.3554534),
        Meters.of(1.12395),
        new Rotation3d(0, 0, Math.PI)),
    new Pose3d(
        Meters.of(4.251134),
        Meters.of(0.0001466),
        Meters.of(1.12395),
        new Rotation3d(0, 0, Math.PI)),
    new Pose3d(
        Meters.of(4.0030014),
        Meters.of(0.603549),
        Meters.of(1.12395),
        new Rotation3d(0, 0, -Math.PI / 2)),
    new Pose3d(
        Meters.of(3.684841), Meters.of(3.3902846), Meters.of(0.889), new Rotation3d(0, 0, Math.PI)),
    new Pose3d(
        Meters.of(8.2453094), Meters.of(3.3707266), Meters.of(0.55245), new Rotation3d(0, 0, 0)),
    new Pose3d(
        Meters.of(8.2453094), Meters.of(2.9389266), Meters.of(0.55245), new Rotation3d(0, 0, 0)),
    new Pose3d(
        Meters.of(8.2449538), Meters.of(0.2913068), Meters.of(0.55245), new Rotation3d(0, 0, 0)),
    new Pose3d(
        Meters.of(8.2449538), Meters.of(-0.1404932), Meters.of(0.55245), new Rotation3d(0, 0, 0))
  };
}
