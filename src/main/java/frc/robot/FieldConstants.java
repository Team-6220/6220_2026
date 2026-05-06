package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Simulation geometry constants to align the sim with the real robot and field.
 *
 * <p>Tune these values to match the actual robot geometry and field layout.
 */
public final class FieldConstants {
  private FieldConstants() {}

  // Field geometry (meters) from 2026 Game Manual section 5.2
  public static final double FIELD_LENGTH_M = 16.54; // 651.2 in
  public static final double FIELD_WIDTH_M = 8.07; // 317.7 in

  // Hub geometry from 2026 Game Manual section 5.4
  public static final double HUB_SIZE_M = 1.19; // 47 in
  public static final double HUB_TOP_OPENING_HEIGHT_M = 1.83; // 72 in (front edge of opening)

  // Hub positions in field coordinates (meters)
  public static final Pose2d BLUE_HUB_POSE = new Pose2d(4.625594, 4.034536, new Rotation2d());
  public static final Pose2d RED_HUB_POSE = new Pose2d(11.915394, 4.034536, new Rotation2d());

  // Hub height (meters) for 3D visualization (use opening height)
  public static final double HUB_HEIGHT_M = HUB_TOP_OPENING_HEIGHT_M;

  // Turret geometry relative to robot center
  public static final Translation2d TURRET_OFFSET_XY = new Translation2d(0.0, 0.0); // meters
  public static final double TURRET_HEIGHT_M = 0.5; // meters above floor

  // Camera geometry
  //Check "WPILIB Coordinate" and MAKE SURE YOU HAVE THIS RIGHT
  public static final Transform3d CAMERA_TO_ROBOT =
      new Transform3d(
          new Translation3d(0.2, 0.0, 0.7), new edu.wpi.first.math.geometry.Rotation3d());

  // If you have multiple cameras, create additional transforms here.

  // AprilTag poses for the 2026 field
  public static final Pose3d[] APRILTAG_POSES = {
    new Pose3d(-3.604959, -3.3899914, 0.889, new Rotation3d(0, 0, Math.PI)),
    new Pose3d(-3.6423986, -0.6032558, 1.12395, new Rotation3d(0, 0, Math.PI / 2)),
    new Pose3d(-3.0388438, -0.3554534, 1.12395, new Rotation3d(0, 0, Math.PI)),
    new Pose3d(-3.0388438, 0.0001466, 1.12395, new Rotation3d(0, 0, Math.PI)),
    new Pose3d(-3.6423986, 0.603549, 1.12395, new Rotation3d(0, 0, -Math.PI / 2)),
    new Pose3d(-3.604959, 3.3902846, 0.889, new Rotation3d(0, 0, Math.PI)),
    new Pose3d(-3.6798636, 3.3902846, 0.889, new Rotation3d(0, 0, 0)),
    new Pose3d(-3.9979986, 0.603549, 1.12395, new Rotation3d(0, 0, -Math.PI / 2)),
    new Pose3d(-4.2461566, 0.3557466, 1.12395, new Rotation3d(0, 0, 0)),
    new Pose3d(-4.2461566, 0.0001466, 1.12395, new Rotation3d(0, 0, 0)),
    new Pose3d(-3.9979986, -0.6032558, 1.12395, new Rotation3d(0, 0, Math.PI / 2)),
    new Pose3d(-3.6798636, -3.3899914, 0.889, new Rotation3d(0, 0, 0)),
    new Pose3d(-8.240332, -3.370408, 0.55245, new Rotation3d(0, 0, Math.PI)),
    new Pose3d(-8.240332, -2.938608, 0.55245, new Rotation3d(0, 0, Math.PI)),
    new Pose3d(-8.2399764, -0.2909882, 0.55245, new Rotation3d(0, 0, Math.PI)),
    new Pose3d(-8.2399764, 0.1408118, 0.55245, new Rotation3d(0, 0, Math.PI)),
    new Pose3d(3.6099364, 3.3902846, 0.889, new Rotation3d(0, 0, 0)),
    new Pose3d(3.6474014, 0.603549, 1.12395, new Rotation3d(0, 0, Math.PI / 2)),
    new Pose3d(3.0438466, 0.3557466, 1.12395, new Rotation3d(0, 0, 0)),
    new Pose3d(3.0438466, 0.0001466, 1.12395, new Rotation3d(0, 0, 0)),
    new Pose3d(3.6474014, -0.6032558, 1.12395, new Rotation3d(0, 0, -Math.PI / 2)),
    new Pose3d(3.6099364, -3.3899914, 0.889, new Rotation3d(0, 0, 0)),
    new Pose3d(3.684841, -3.3899914, 0.889, new Rotation3d(0, 0, Math.PI)),
    new Pose3d(4.0030014, -0.6032558, 1.12395, new Rotation3d(0, 0, Math.PI / 2)),
    new Pose3d(4.251134, -0.3554534, 1.12395, new Rotation3d(0, 0, Math.PI)),
    new Pose3d(4.251134, 0.0001466, 1.12395, new Rotation3d(0, 0, Math.PI)),
    new Pose3d(4.0030014, 0.603549, 1.12395, new Rotation3d(0, 0, -Math.PI / 2)),
    new Pose3d(3.684841, 3.3902846, 0.889, new Rotation3d(0, 0, Math.PI)),
    new Pose3d(8.2453094, 3.3707266, 0.55245, new Rotation3d(0, 0, 0)),
    new Pose3d(8.2453094, 2.9389266, 0.55245, new Rotation3d(0, 0, 0)),
    new Pose3d(8.2449538, 0.2913068, 0.55245, new Rotation3d(0, 0, 0)),
    new Pose3d(8.2449538, -0.1404932, 0.55245, new Rotation3d(0, 0, 0))
  };
}
