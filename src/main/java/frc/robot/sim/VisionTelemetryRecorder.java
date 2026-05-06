package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;

/**
 * Utility to record and replay camera/vision pose measurements for hybrid simulations.
 *
 * <p>File format (CSV): timestamp,x,y,deg Example: 12.340,1.234,2.345,45.0
 */
public final class VisionTelemetryRecorder {
  private final File file;
  private final List<Entry> playback = new ArrayList<>();

  public VisionTelemetryRecorder(String path) {
    this.file = new File(path);
  }

  public synchronized void record(Pose2d pose, double timestampSeconds) {
    try (BufferedWriter w = new BufferedWriter(new FileWriter(file, true))) {
      w.write(
          String.format(
              "%.3f,%.6f,%.6f,%.3f\n",
              timestampSeconds, pose.getX(), pose.getY(), pose.getRotation().getDegrees()));
    } catch (Exception ex) {
      // ignore
    }
  }

  /** Load all records from file for replay. */
  public synchronized void loadPlayback() {
    playback.clear();
    if (!file.exists()) {
      return;
    }
    try (BufferedReader r = new BufferedReader(new FileReader(file))) {
      String line;
      while ((line = r.readLine()) != null) {
        if (line.isBlank() || line.startsWith("#")) {
          continue;
        }
        String[] parts = line.trim().split(",");
        if (parts.length >= 4) {
          double t = Double.parseDouble(parts[0]);
          double x = Double.parseDouble(parts[1]);
          double y = Double.parseDouble(parts[2]);
          double deg = Double.parseDouble(parts[3]);
          playback.add(new Entry(t, new Pose2d(x, y, Rotation2d.fromDegrees(deg))));
        }
      }
    } catch (Exception ex) {
      // ignore
    }
  }

  /** Returns the last pose measurement with timestamp <= t (or null if none). */
  public synchronized Pose2d getPoseAtTime(double timestampSeconds) {
    Pose2d last = null;
    for (Entry e : playback) {
      if (e.timestamp <= timestampSeconds) {
        last = e.pose;
      } else {
        break;
      }
    }
    return last;
  }

  private static class Entry {
    final double timestamp;
    final Pose2d pose;

    Entry(double t, Pose2d p) {
      timestamp = t;
      pose = p;
    }
  }
}
