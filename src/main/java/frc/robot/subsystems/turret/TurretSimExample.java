package frc.robot.subsystems.turret;

import edu.wpi.first.math.controller.PIDController;

/**
 * Small example that demonstrates how to use TurretPhysicsSim. This is not a Robot program; it's a
 * helper class you can call from unit tests or a main method.
 */
public final class TurretSimExample {
  private TurretSimExample() {}

  /** Run a short simulation where a PID controller drives the turret to a target angle. */
  /** Run a short simulation where a PID controller drives the turret to a target angle. */
  @SuppressWarnings("resource")
  public static SimulationResult runExample() {
    TurretPhysicsSim sim = new TurretPhysicsSim();
    // PIDController is not closeable here; avoid patterns that some linters treat as resources.
    PIDController pid = new PIDController(8.0, 0.0, 0.5);
    double dt = 0.02; // 20ms update
    double target = Math.toRadians(45.0);

    sim.reset(0.0, 0.0);

    for (int i = 0; i < 1500; i++) { // 30s max
      double error = target - sim.getAngleRad();
      double cmd = pid.calculate(sim.getAngleRad(), target);
      // Convert PID output (assumed in radians) to volts using a simple gain. In a real system
      // you'd use a velocity/position-controlling loop or feedforward. This is a simple demo.
      double volts = Math.max(-12.0, Math.min(12.0, cmd * 6.0));
      sim.setVoltage(volts);
      sim.update(dt);
      if (Math.abs(error) < Math.toRadians(0.5) && Math.abs(sim.getAngularVelocity()) < 0.01) {
        return new SimulationResult(true, sim.getAngleRad(), sim.getAngularVelocity(), i * dt);
      }
    }
    return new SimulationResult(false, sim.getAngleRad(), sim.getAngularVelocity(), 1500 * dt);
  }

  public static final class SimulationResult {
    public final boolean success;
    public final double finalAngleRad;
    public final double finalAngularVelocity;
    public final double timeSeconds;

    public SimulationResult(boolean success, double a, double w, double t) {
      this.success = success;
      this.finalAngleRad = a;
      this.finalAngularVelocity = w;
      this.timeSeconds = t;
    }
  }

  /** Simple main so you can run the example directly. */
  public static void main(String[] args) {
    SimulationResult r = runExample();
    System.out.println(
        "TurretSimExample result:\n  success="
            + r.success
            + "\n  finalAngleDeg="
            + Math.toDegrees(r.finalAngleRad)
            + "\n  finalAngularVelocity="
            + r.finalAngularVelocity
            + "\n  timeSeconds="
            + r.timeSeconds);
  }
}
