package frc.robot.subsystems.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A simulated turret subsystem that wraps the TurretPhysicsSim and provides a simple API compatible
 * with the project's existing TurretSubsystem: setTargetAngle, getAngle, atSetpoint, and a periodic
 * update method.
 */
public class TurretSubsystemSim extends SubsystemBase implements TurretSubsystemIO {
  private final TurretPhysicsSim sim;
  private final PIDController pid;
  private final double encoderCPR; // counts per revolution at the encoder (before gearbox)
  private final double gearRatio; // motor revs per turret rev

  private double lastTarget = 0.0;

  public TurretSubsystemSim() {
    this(new TurretPhysicsSim(), new PIDController(4.0, 0.0, 0.1), 4096.0, 10.0);
  }

  public TurretSubsystemSim(
      TurretPhysicsSim sim, PIDController pid, double encoderCPR, double gearRatio) {
    this.sim = sim;
    this.pid = pid;
    this.encoderCPR = encoderCPR;
    this.gearRatio = gearRatio;
  }

  /** Command raw motor voltage (V) to the sim. */
  public void setMotorVoltage(double volts) {
    sim.setVoltage(volts);
  }

  /**
   * Closed-loop: set a target angle (robot/turret frame) — uses internal PID to compute voltage.
   */
  public void setTargetAngle(Rotation2d angle) {
    lastTarget = angle.getRadians();
    double pidOut = pid.calculate(sim.getAngleRad(), lastTarget);
    // Convert PID output to volts using a modest scaling. This is intentionally simple —
    // replace with a proper controller when integrating with real hardware.
    double volts = Math.max(-12.0, Math.min(12.0, pidOut * 6.0));
    setMotorVoltage(volts);
  }

  /** Advance the simulation by dt seconds. Call from your robot loop or test harness. */
  public void update(double dtSeconds) {
    sim.update(dtSeconds);
  }

  public Rotation2d getAngle() {
    return new Rotation2d(sim.getAngleRad());
  }

  public boolean atSetpoint() {
    return Math.abs(sim.getAngleRad() - lastTarget) < Math.toRadians(0.5);
  }

  /** Returns encoder counts for the turret position (simulated). */
  public double getEncoderCounts() {
    // turret revolutions -> motor revolutions = gearRatio * turretRev
    double turretRevs = sim.getAngleRad() / (2.0 * Math.PI);
    double motorRevs = turretRevs * gearRatio;
    return motorRevs * encoderCPR;
  }

  public double getAngularVelocity() {
    return sim.getAngularVelocity();
  }

  /** Reset the simulation state. */
  public void reset(double angleRad, double angVel) {
    sim.reset(angleRad, angVel);
    pid.reset();
    lastTarget = angleRad;
  }
}
