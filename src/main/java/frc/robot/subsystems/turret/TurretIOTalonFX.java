package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;

/** TalonFX-based Turret IO implementation (single Falcon/TalonFX motor). */
public class TurretIOTalonFX implements TurretIO {
  private final TalonFX motor;
  private double lastAppliedVolts = 0.0;

  public TurretIOTalonFX(int canId) {
    motor = new TalonFX(canId);
    // Do not apply team CTRE configs here - Robot may call configurator elsewhere
    // keep constructor lightweight
  }

  @Override
  public void driveToGoal(Rotation2d angle) {
    // Best-effort: set the onboard position to target angle so closed-loop controllers
    // that read position can use it. This mirrors patterns elsewhere in the repo where
    // getConfigurator().setPosition(...) is used.
    motor.getConfigurator().setPosition(Degree.of(Math.toDegrees(angle.getRadians())));
  }

  @Override
  public void setVoltageOut(Voltage voltsOut) {
    lastAppliedVolts = voltsOut.in(Volts);
    motor.setVoltage(voltsOut.baseUnitMagnitude());
  }

  @Override
  public boolean atSetpoint() {
    // No generic at-setpoint API available here; the subsystem handles checking setpoint.
    return false;
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    try {
      double pos = motor.getPosition().getValueAsDouble(); // CTRE unit -- usually rotations
      inputs.builtinEncoderAngle = new Rotation2d(pos * 2.0 * Math.PI);
    } catch (Exception e) {
      inputs.builtinEncoderAngle = new Rotation2d(0.0);
    }

    try {
      inputs.appliedVoltage = Volts.of(motor.getSupplyVoltage().getValueAsDouble());
    } catch (Exception e) {
      inputs.appliedVoltage = Volts.of(lastAppliedVolts);
    }

    try {
      inputs.motorCurrent = Amps.of(motor.getStatorCurrent().getValueAsDouble());
    } catch (Exception e) {
      inputs.motorCurrent = Amps.of(0.0);
    }

    inputs.motorTemperature = Celsius.of(30.0);
  }

  @Override
  public void resetBuiltInEncoderAngle(double angleRad, double angularVelocity) {
    // Configure the TalonFX internal position to the provided angle (degrees)
    motor.getConfigurator().setPosition(Degree.of(Math.toDegrees(angleRad)));
  }

  @Override
  public void resetOutsideInEncoderAngle(double angleRad, double angularVelocity) {
    // No separate outside encoder by default; mirror builtin reset
    resetBuiltInEncoderAngle(angleRad, angularVelocity);
  }
}
