package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Shared turret interface for real and simulated implementations. */
public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public Rotation2d builtinEncoderAngle;
    public Rotation2d outsideEncoderAngle;
    public Temperature motorTemperature;
    public Current motorCurrent;
    public Voltage appliedVoltage;
  }

  void driveToGoal(Rotation2d angle);
  
  void setVoltageOut(Voltage voltsOut);

  boolean atSetpoint();


  default void updateInputs(TurretIOInputs inputs) {}

  /** Optional reset for sims/hardware. */
  default void resetBuiltInEncoderAngle(double angleRad, double angularVelocity) {}

  default void resetOutsideInEncoderAngle(double angleRad, double angularVelocity) {}
}
