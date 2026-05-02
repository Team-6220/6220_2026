package frc.robot.tracking;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayDeque;
import java.util.Deque;

/**
 * Simple sliding-window target estimator that stores the most recent observations and computes a
 * constant-velocity estimate.
 */
public class TargetEstimator {
  private static final int DEFAULT_WINDOW = 6;

  private final Deque<Entry> window = new ArrayDeque<>();
  private final int capacity;

  public TargetEstimator() {
    this(DEFAULT_WINDOW);
  }

  public TargetEstimator(int windowSize) {
    this.capacity = Math.max(2, windowSize);
  }

  public synchronized void addObservation(Pose2d pose, double timestampSeconds) {
    window.addLast(new Entry(pose, timestampSeconds));
    while (window.size() > capacity) {
      window.removeFirst();
    }
  }

  /**
   * Returns estimated target velocity in meters per second in field coordinates as a Translation2d
   * (vx, vy). Returns zero when insufficient data exists.
   */
  public synchronized Translation2d estimateVelocity() {
    if (window.size() < 2) {
      return new Translation2d(0, 0);
    }
    Entry first = window.getFirst();
    Entry last = window.getLast();
    double dt = last.t - first.t;
    if (dt <= 1e-6) {
      return new Translation2d(0, 0);
    }
    double dx = last.pose.getX() - first.pose.getX();
    double dy = last.pose.getY() - first.pose.getY();
    return new Translation2d(dx / dt, dy / dt);
  }

  private static class Entry {
    final Pose2d pose;
    final double t;

    Entry(Pose2d p, double ts) {
      pose = p;
      t = ts;
    }
  }
}
