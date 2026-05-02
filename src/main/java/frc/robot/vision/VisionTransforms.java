package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

/** Utilities to convert between camera-space and robot-space poses. */
public final class VisionTransforms {
  private VisionTransforms() {}

  /**
   * Convert a target pose given in camera coordinates (Pose3d) into a 2D robot-relative Pose2d.
   *
   * @param cameraToTarget Pose3d of the target in the camera frame
   * @param cameraToRobot Transform3d describing where the camera is mounted on the robot (camera
   *     relative to robot)
   * @return Pose2d of the target in the robot frame (x forward, y left)
   */
  public static Pose2d cameraToRobotPose2d(Pose3d cameraToTarget, Transform3d cameraToRobot) {
    // cameraToTarget is the pose of the target in camera coordinates. To get target in robot
    // coordinates we first compute camera_in_robot * target_in_camera.
    // Transform3d provides transform multiplication as: cameraToRobot.transform(cameraToTarget)
    // but we can apply by converting Pose3d to Translation3d and Rotation.
    // Get translations and yaw angles
    var camToRobotTrans = cameraToRobot.getTranslation();
    var camToRobotRot = cameraToRobot.getRotation();
    var targetTrans = cameraToTarget.getTranslation();
    var targetRot = cameraToTarget.getRotation();

    // Compute yaw angles (around Z)
    double camYaw = camToRobotRot.getZ();
    double targetYaw = targetRot.getZ();

    // Rotate target (x,y) by camera yaw, then translate by camera position
    double xInRobot =
        camToRobotTrans.getX()
            + (targetTrans.getX() * Math.cos(camYaw) - targetTrans.getY() * Math.sin(camYaw));
    double yInRobot =
        camToRobotTrans.getY()
            + (targetTrans.getX() * Math.sin(camYaw) + targetTrans.getY() * Math.cos(camYaw));

    Rotation2d rot = new Rotation2d(camYaw + targetYaw);

    return new Pose2d(new Translation2d(xInRobot, yInRobot), rot);
  }
}
