package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VisionConstants;

public class PhotonVisionSubsystem extends SubsystemBase {
  private static PhotonCamera[] cameras;
  private static String[] cameraNames;
  private final long[] lastHeartbeats;

  public static Field2d theFieldCam0 = new Field2d(), theFieldCam1 = new Field2d();

  private PhotonTrackedTarget noErrorHopefully;

  private static PhotonVisionSubsystem INSTANCE = null;

  private static String tableKey = "Vision_";

  private HashMap<Integer, List<PhotonPipelineResult>> results;
  private HashMap<Integer, List<PhotonTrackedTarget>> bestTarget;

  /** Creates a new PhotonVisionSubsystem. */
  public PhotonVisionSubsystem(String[] cameraNames) {
    cameras = new PhotonCamera[cameraNames.length];
    lastHeartbeats = new long[cameraNames.length];
    this.cameraNames = cameraNames;

    initPhoton();

    results = new HashMap<Integer, List<PhotonPipelineResult>>();
    for (int i = 0; i < cameras.length; i++) {
      results.put(i, null);
    }

    bestTarget = new HashMap<Integer, List<PhotonTrackedTarget>>();
    for (int i = 0; i < cameras.length; i++) {
      bestTarget.put(i, null);
    }
  }

  @Override
  public void periodic() {}

  public void initPhoton() {
    for (int i = 0; i < cameraNames.length; i++) {
      if (isCameraConnected(cameraNames[i])) {
        cameras[i] = new PhotonCamera(cameraNames[i]);
        System.out.println("Photon camera initialized: " + cameraNames[i]);
        cameras[i].setPipelineIndex(0);
        NetworkTable camTable =
            NetworkTableInstance.getDefault().getTable("photonvision/" + cameras[i].getName());
        NetworkTableEntry heartbeatEntry = camTable.getEntry("heartbeat");
        lastHeartbeats[i] = (long) heartbeatEntry.getDouble(-1);
      } else {
        cameras[i] = null;
        System.out.println("Photon camera not found: " + cameraNames[i]);
        lastHeartbeats[i] = -1;
      }
    }
  }

  public void updatePhoton() {
    for (int i = 0; i < cameras.length; i++) {
      if (cameras[i] == null) {
        results.put(i, null);
        bestTarget.put(i, new ArrayList<>());
        System.err.println(cameraNames[i] + " isNull");
        continue;
      }

      NetworkTable camTable =
          NetworkTableInstance.getDefault().getTable("photonvision/" + cameras[i].getName());
      NetworkTableEntry heartbeatEntry = camTable.getEntry("heartbeat");

      if (!heartbeatEntry.exists()) {
        results.put(i, null);
        bestTarget.put(i, new ArrayList<>());
        System.err.println(cameraNames[i] + "doesn't have a heartbeat entry");
        continue;
      }

      long currentHeartbeat = (long) heartbeatEntry.getDouble(-1);
      if (currentHeartbeat == lastHeartbeats[i]) {
        // Heartbeat hasn't changed → camera likely stalled or unplugged
        results.put(i, null);
        bestTarget.put(i, new ArrayList<>());
        System.err.println(cameraNames[i] + "heartbeat stayed the same");
        continue;
      }

      lastHeartbeats[i] = currentHeartbeat;

      List<PhotonPipelineResult> unreadResults = cameras[i].getAllUnreadResults();
      if (!unreadResults.isEmpty()) {
        results.put(i, unreadResults);
      } else {
        System.err.println(cameraNames[i] + "pipeline is empty");
        continue;
      }
      if (!results.isEmpty()) {
        bestTarget.put(i, results.get(i).get(0).getTargets());
      }
    }
  }

  private boolean isCameraConnected(String cameraName) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/" + cameraName);
    return table.getKeys().size() > 0;
  }

  public PhotonCamera[] getCameras() {
    return cameras;
  }

  public HashMap<Integer, List<PhotonPipelineResult>> getResults() {
    return results;
  }

  public HashMap<Integer, List<PhotonTrackedTarget>> getBestTargets() {
    return bestTarget;
  }

  public static synchronized PhotonVisionSubsystem getInstance(String[] cameraNames) {
    if (INSTANCE == null) {
      INSTANCE = new PhotonVisionSubsystem(cameraNames);
    }
    return INSTANCE;
  }
}
