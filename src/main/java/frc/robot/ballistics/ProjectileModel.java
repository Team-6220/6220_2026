package frc.robot.ballistics;

/**
 * Very small projectile model for horizontal-range time-of-flight estimates. Assumes flat
 * trajectory and constant initial speed; drag is ignored. This is intentionally simple and intended
 * to be replaced or tuned with empiric data.
 */
public final class ProjectileModel {
  private ProjectileModel() {}

  /**
   * Estimate time-of-flight assuming constant horizontal speed v (m/s): t = range / v
   *
   * @param rangeMeters horizontal distance in meters
   * @param initialSpeedMps projectile speed in m/s
   * @return seconds (double), or Double.POSITIVE_INFINITY if initialSpeedMps <= 0
   */
  public static double estimateTimeOfFlight(double rangeMeters, double initialSpeedMps) {
    if (initialSpeedMps <= 0) {
      return Double.POSITIVE_INFINITY;
    }
    return Math.abs(rangeMeters) / initialSpeedMps;
  }
}
