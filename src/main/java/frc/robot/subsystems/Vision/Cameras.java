// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import com.limelightvision.Limelight;
import com.limelightvision.LimelightResults;
import com.limelightvision.PoseEstimateConfig;

/** Shared LimelightLib 2 camera instances. */
public final class Cameras {
  private Cameras() {}

  // TODO: Measure the physical mount and replace these placeholders. Offsets are from the
  // robot's origin (center of the drivetrain, floor height) to the camera lens.
  // LimelightLib 2 uses right-handed NWU: X forward, Y left, Z up. Positive pitch tilts the
  // camera DOWN, so a camera tilted up 55 degrees is -55. (LimelightHelpers used Y right and
  // positive pitch = up, so side and pitch signs are flipped compared to the old values.)
  private static final double CAMERA_FORWARD_METERS = 0.0;
  private static final double CAMERA_LEFT_METERS = 0.0;
  private static final double CAMERA_UP_METERS = 0.0;
  private static final double CAMERA_ROLL_DEGREES = 0.0;
  private static final double CAMERA_PITCH_DEGREES = -55;
  private static final double CAMERA_YAW_DEGREES = 0.0;

  // MegaTag2 filtering and trust scaling. Starting values from the LimelightLib 2 docs; tune on
  // the robot.
  private static final PoseEstimateConfig MT2_CONFIG =
      PoseEstimateConfig.defaultMT2()
          .withMinTagCount(1)
          .withMaxSingleTagAmbiguity(1.0) // MT2 handles ambiguous perspectives
          .withMaxSingleTagDistance(0.0) // 0 disables this check
          .withMaxAvgTagDistance(8.0)
          .withMinAvgTagArea(0.02) // 0-100, percentage of image area
          .withFieldBounds(16.541, 8.069) // 2026 welded field
          .withFieldBoundsMargin(0.5)
          .withStdDevXY(0.3, 0.0001, 2.0)
          .withStdDevTheta(
              PoseEstimateConfig.UNTRUSTED,
              PoseEstimateConfig.UNTRUSTED,
              PoseEstimateConfig.UNTRUSTED) // never fuse vision heading
          .withStdDevDistanceScaling(0.5, 0.0, 8.0) // scale by sqrt(distance)
          .withStdDevTagCountDivision(0.5); // trust grows with sqrt(tag count)

  /** Front-facing Limelight used for aiming and MegaTag2 localization. */
  public static final Limelight FRONT =
      new Limelight(
              "limelight-front",
              CAMERA_FORWARD_METERS,
              CAMERA_LEFT_METERS,
              CAMERA_UP_METERS,
              CAMERA_ROLL_DEGREES,
              CAMERA_PITCH_DEGREES,
              CAMERA_YAW_DEGREES)
          .withPoseEstimateConfig_MT2(MT2_CONFIG);

  static {
    FRONT.setPipelineIndex(0);
  }

  /**
   * Gets the ID of the primary visible AprilTag on the front camera.
   *
   * @return The tag ID, or -1 if no tag is visible
   */
  public static int getFrontTagID() {
    if (!FRONT.hasTarget()) return -1;
    LimelightResults results = FRONT.getLatestResults();
    if (results.fiducialTargets == null || results.fiducialTargets.length == 0) return -1;
    return results.fiducialTargets[0].fiducialId;
  }

  /** Switches the front camera to the pipeline for the currently visible tag. */
  public static void setPipelineForVisibleTag() {
    int tag = getFrontTagID();
    if (tag == 2 || tag == 18) {
      FRONT.setPipelineIndex(1);
    } else if (tag == 11 || tag == 27) {
      FRONT.setPipelineIndex(2);
    } else if (tag == 10 || tag == 26) {
      FRONT.setPipelineIndex(3);
    } else if (tag == 9 || tag == 25) {
      FRONT.setPipelineIndex(4);
    } else if (tag == 8 || tag == 24) {
      FRONT.setPipelineIndex(5);
    } else if (tag == 5 || tag == 21) {
      FRONT.setPipelineIndex(6);
    } else {
      FRONT.setPipelineIndex(0);
    }
  }
}
