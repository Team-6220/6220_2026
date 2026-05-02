package frc.robot.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Minimal turret subsystem with a PID controller for angle. Meant as a compile-ready stub. */
public class TurretSubsystem extends SubsystemBase {
  private final PIDController pid = new PIDController(4.0, 0, 0.1);
  private double currentAngle = 0.0; // radians

  public void setTargetAngle(Rotation2d angle) {
    double target = angle.getRadians();
    double output = pid.calculate(currentAngle, target);
    // In real code: write to motor with feedforward/voltage limits
    currentAngle += output * 0.02; // simulate a step
  }

  public boolean atSetpoint() {
    return pid.atSetpoint();
  }

  public Rotation2d getAngle() {
    return new Rotation2d(currentAngle);
  }
}
