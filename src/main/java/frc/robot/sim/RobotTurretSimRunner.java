package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.aiming.AimPredictor;
import frc.robot.aiming.AimSolution;
import frc.robot.subsystems.tracking.TargetEstimator;
import frc.robot.subsystems.turret.TurretSubsystemSim;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.time.Instant;
import java.util.Locale;

/**
 * Simple offline simulation runner that advances a robot pose, turret sim, and target estimator.
 *
 * <p>Inputs are read from a plain text file `sim_inputs.txt` (in repo root) with a single line
 * containing: forward_m_s strafe_m_s omega_rad_s alliance e.g. "1.0 0.0 0.0 blue" to drive forward
 * at 1 m/s on the blue alliance.
 *
 * <p>Outputs are constantly written to `sim_output_pose3d.txt` (overwritten each tick) with a
 * human-readable Pose3d for robot and turret.
 *
 * <p>Assumptions made (tunable): - Field hubs are at ±8 meters along the X axis (blue at +8, red at
 * -8), y=0. Adjust in code. - Turret angle is expressed in robot frame (0 points along robot X
 * axis). We compute the turret's absolute pose by adding turret yaw to robot yaw.
 */
public final class RobotTurretSimRunner {
  // Files
  private static final String INPUT_FILE = "sim_inputs.txt";
  private static final String OUTPUT_FILE = "sim_output_pose3d.txt";
  private static final String LATEST_FILE = "sim_latest_pose3d.txt";
  private static final String VISION_RECORD_FILE = "sim_vision_recording.csv";

  private final TurretSubsystemSim turret;
  private final TargetEstimator targetEstimator;
  private final SimPoseEstimator poseEstimator;
  private final SimRangeEstimator rangeEstimator;
  private final VisionTelemetryRecorder visionRecorder;
  private final NetworkTable simTable;

  // robot ground truth pose (field frame)
  private double x = 0.0;
  private double y = 0.0;
  private double yaw = 0.0; // radians

  private String alliance = "blue";
  private boolean recordVision = false;
  private boolean replayVision = false;

  private XboxController xboxcontroller;

  public RobotTurretSimRunner(XboxController controller) {
    turret = new TurretSubsystemSim();
    targetEstimator = new TargetEstimator();
    poseEstimator = new SimPoseEstimator();
    rangeEstimator = new SimRangeEstimator();
    visionRecorder = new VisionTelemetryRecorder(VISION_RECORD_FILE);
    simTable = NetworkTableInstance.getDefault().getTable("Sim").getSubTable("Turret");
    xboxcontroller = controller;
  }

  public void runLoop(double totalSeconds) throws Exception {
    double dt = 0.02; // 20ms
    int steps = (int) Math.ceil(totalSeconds / dt);

    for (int i = 0; i < steps; i++) {
      double t = i * dt;
      // read inputs (non-blocking)
      readInputs();

      // read desired velocities
      double forward = xboxcontroller.getLeftY();
      double strafe = xboxcontroller.getLeftX();
      double omega = xboxcontroller.getRightX();

      // // Try to read first line of input file
      // File f = new File(INPUT_FILE);
      // if (f.exists()) {
      //   try (BufferedReader r = new BufferedReader(new FileReader(f))) {
      //     String line = r.readLine();
      //     if (line != null && !line.isBlank()) {
      //       String[] parts = line.trim().split("\\s+");
      //       if (parts.length >= 4) {
      //         forward = Double.parseDouble(parts[0]);
      //         strafe = Double.parseDouble(parts[1]);
      //         omega = Double.parseDouble(parts[2]);
      //         alliance = parts[3].toLowerCase(Locale.ROOT);
      //         if (parts.length >= 6) {
      //           recordVision = Boolean.parseBoolean(parts[4]);
      //           replayVision = Boolean.parseBoolean(parts[5]);
      //         }
      //       }
      //     }
      //   } catch (Exception ex) {
      //     // ignore malformed input and use zeros
      //   }
      // }

      // Integrate robot pose in field frame (simple kinematic model: robot field velocities)
      // Rotate body-frame velocities into field frame
      double cos = Math.cos(yaw);
      double sin = Math.sin(yaw);
      double vxField = forward * cos - strafe * sin;
      double vyField = forward * sin + strafe * cos;
      x += vxField * dt;
      y += vyField * dt;
      yaw += omega * dt;

      // Determine which hub to aim at
      Pose2d hubPose =
          alliance.equals("blue")
              ? SimGeometryConstants.BLUE_HUB_POSE
              : SimGeometryConstants.RED_HUB_POSE;

      // Update pose estimator with simulated ground truth
      poseEstimator.update(new Pose2d(x, y, new Rotation2d(yaw)));
      rangeEstimator.setTarget(hubPose.getX(), hubPose.getY());

      // Hybrid vision: record actual camera measurements or replay logged data
      double ts = Instant.now().toEpochMilli() / 1000.0;
      if (replayVision) {
        if (i == 0) {
          visionRecorder.loadPlayback();
        }
        Pose2d replayPose = visionRecorder.getPoseAtTime(ts);
        if (replayPose != null) {
          targetEstimator.addObservation(replayPose, ts);
        } else {
          targetEstimator.addObservation(hubPose, ts);
        }
      } else {
        targetEstimator.addObservation(hubPose, ts);
        if (recordVision) {
          visionRecorder.record(hubPose, ts);
        }
      }

      // Compute shooter pose (turret center) using robot pose and turret offset
      Translation2d offset = SimGeometryConstants.TURRET_OFFSET_XY;
      double offX = offset.getX() * Math.cos(yaw) - offset.getY() * Math.sin(yaw);
      double offY = offset.getX() * Math.sin(yaw) + offset.getY() * Math.cos(yaw);
      Pose2d shooterPose = new Pose2d(x + offX, y + offY, new Rotation2d(yaw));

      // Use target estimator velocity to predict intercept
      Translation2d targetVel = targetEstimator.estimateVelocity();
      AimSolution sol =
          AimPredictor.predictIntercept(
              hubPose, targetVel, shooterPose, SimGeometryConstants.PROJECTILE_SPEED_MPS);
      double turretAngleRobotFrame = sol.requiredTurretAngleRad;
      turret.setTargetAngle(new Rotation2d(turretAngleRobotFrame));

      // step turret sim
      turret.update(dt);

      // Build Pose3d outputs
      // Build Rotation3d from yaw
      edu.wpi.first.math.geometry.Rotation3d robotRot3 =
          new edu.wpi.first.math.geometry.Rotation3d(0.0, 0.0, yaw);
      Pose3d robotPose3d = new Pose3d(x, y, 0.0, robotRot3);
      // turret pose3d: assume turret rotates about robot center; give turret yaw in field frame
      double turretYawField = yaw + turret.getAngle().getRadians();
      edu.wpi.first.math.geometry.Rotation3d turretRot3 =
          new edu.wpi.first.math.geometry.Rotation3d(0.0, 0.0, turretYawField);
      Pose3d turretPose3d =
          new Pose3d(x + offX, y + offY, SimGeometryConstants.TURRET_HEIGHT_M, turretRot3);

      // compute relative vector to hub (field frame)
      double relX = hubPose.getX() - x;
      double relY = hubPose.getY() - y;
      double relDist = Math.hypot(relX, relY);
      double relAngleField = Math.atan2(relY, relX);
      double relAngleRobot = relAngleField - yaw;

      // estimated target velocity from TargetEstimator (field frame)
      edu.wpi.first.math.geometry.Translation2d estVel = targetVel;

      // write outputs (robot/turret pose + relative info + est velocity + poseEstimator/range)
      writeOutput(
          robotPose3d,
          turretPose3d,
          t,
          relX,
          relY,
          relDist,
          relAngleRobot,
          estVel.getX(),
          estVel.getY(),
          poseEstimator.getEstimatedPosition(),
          rangeEstimator.estimateRange(x, y));

      // Log to NetworkTables for live view
      publishNetworkTables(robotPose3d, turretPose3d, relX, relY, relDist, relAngleRobot, estVel);

      // small sleep to pace (in real usage you might run faster or step without sleeping)
      Thread.sleep((long) (dt * 1000));
    }
  }

  private void readInputs() {
    // Placeholder: future extension to read live joystick or networked inputs.
  }

  private void publishNetworkTables(
      Pose3d robotPose,
      Pose3d turretPose,
      double relX,
      double relY,
      double relDist,
      double relAngleRobot,
      Translation2d estVel) {
    simTable.getEntry("robotPose").setString(robotPose.toString());
    simTable.getEntry("turretPose").setString(turretPose.toString());
    simTable.getEntry("relX").setDouble(relX);
    simTable.getEntry("relY").setDouble(relY);
    simTable.getEntry("relDist").setDouble(relDist);
    simTable.getEntry("relAngleRobot").setDouble(relAngleRobot);
    simTable.getEntry("targetVelX").setDouble(estVel.getX());
    simTable.getEntry("targetVelY").setDouble(estVel.getY());
  }

  private void writeOutput(
      Pose3d robotPose,
      Pose3d turretPose,
      double t,
      double relX,
      double relY,
      double relDist,
      double relAngleRobot,
      double estVx,
      double estVy,
      edu.wpi.first.math.geometry.Pose2d poseEst,
      double rangeEst) {
    String out =
        String.format(
            Locale.ROOT,
            "t=%.3f\nrobot=%s\nturret=%s\nrelative_to_hub: dx=%.3f dy=%.3f dist=%.3f relAngleRobot=%.3f\nestimated_target_vel: vx=%.3f vy=%.3f\nposeEstimator=%s\nrangeEstimator=%.3f\n\n",
            t,
            robotPose.toString(),
            turretPose.toString(),
            relX,
            relY,
            relDist,
            relAngleRobot,
            estVx,
            estVy,
            poseEst.toString(),
            rangeEst);
    try (BufferedWriter w = new BufferedWriter(new FileWriter(OUTPUT_FILE, false))) {
      w.write(out);
    } catch (Exception ex) {
      // ignore
    }
    try (BufferedWriter w2 = new BufferedWriter(new FileWriter(LATEST_FILE, false))) {
      w2.write(out);
    } catch (Exception ex) {
      // ignore
    }
  }

  // public static void main(String[] args) throws Exception {
  //   RobotTurretSimRunner r = new RobotTurretSimRunner(new XboxController(0));
  //   double total = 60.0; // default 60s
  //   if (args.length >= 1) {
  //     try {
  //       total = Double.parseDouble(args[0]);
  //     } catch (Exception ex) {
  //     }
  //   }
  //   System.out.println(
  //       "Starting RobotTurretSimRunner for "
  //           + total
  //           + "s. Edit '"
  //           + INPUT_FILE
  //           + "' to drive the sim.");
  //   r.runLoop(total);
  //   System.out.println("Simulation finished. See '" + OUTPUT_FILE + "' and '" + LATEST_FILE +
  // "'.");
  // }
}
