package frc.robot.sim;

/**
 * Simple range estimator for simulation: computes Euclidean distance to a known target. In the real
 * system, replace with your vision/range estimation pipeline.
 */
public final class SimRangeEstimator {
  private double targetX = 0.0;
  private double targetY = 0.0;

  public void setTarget(double x, double y) {
    this.targetX = x;
    this.targetY = y;
  }

  public double estimateRange(double robotX, double robotY) {
    double dx = targetX - robotX;
    double dy = targetY - robotY;
    return Math.hypot(dx, dy);
  }
}
