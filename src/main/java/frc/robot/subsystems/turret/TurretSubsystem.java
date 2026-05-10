package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;

/** Turret subsystem with feedforward support. */
public class TurretSubsystem extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputs inputs = new TurretIOInputs();

  // Feedforward computed in subsystem using constants
  private final SimpleMotorFeedforward feedforward = TurretConstants.TURRET_FEEDFORWARD;

  public TurretSubsystem(TurretIO io) {
    this.io = io;
  }

  /** Basic position-only command; delegates to IO. */
  public void driveToGoal(Rotation2d angle) {
    io.driveToGoal(angle);
  }

  /** Returns whether the underlying IO reports being at its setpoint. */
  public boolean atSetpoint() {
    return io.atSetpoint();
  }

  public void setVoltageOut(Voltage voltsOut) {
    io.setVoltageOut(voltsOut);
  }

  /**
   * Convenience helper: move to angle using a feedforward for the target velocity.
   *
   * @param angle target turret angle
   * @param targetVelocityRadPerSec expected turret angular velocity (rad/s) used by feedforward
   */
  public void aimWithFeedforward(Rotation2d angle, double targetVelocityRadPerSec) {
    // Compute feedforward (units: volts)
    double ffVolts = feedforward.calculate(targetVelocityRadPerSec);
    // Delegate position set via IO and apply feedforward voltage
    io.driveToGoal(angle);
    io.setVoltageOut(Volts.of(ffVolts));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  /**
   * Returns the most recently-updated robot-relative turret angle reported by the IO layer. This is
   * the "outside"/relative encoder reading that tracks turret rotation relative to the robot frame.
   * If no reading is available yet, returns zero.
   */
  public Rotation2d getMeasuredRobotRelativeAngle() {
    return inputs.outsideEncoderAngle == null ? new Rotation2d() : inputs.outsideEncoderAngle;
  }
}
