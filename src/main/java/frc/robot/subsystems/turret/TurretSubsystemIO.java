package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;

/** Shared turret interface for real and simulated implementations. */
public interface TurretSubsystemIO {
  void setTargetAngle(Rotation2d angle);

  Rotation2d getAngle();

  boolean atSetpoint();

  /** Optional periodic simulation update. */
  default void update(double dtSeconds) {}

  /** Optional reset for sims/hardware. */
  default void reset(double angleRad, double angularVelocity) {}
}
