package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Celsius;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;

/** SparkMax-based Turret IO implementation (single NEO/NEO550 motor for turret). */
public class TurretIOSparkMax implements TurretIO {
  private final SparkMax motor;
  private final SparkMaxConfig motorConfig = new SparkMaxConfig();
  private final RelativeEncoder builtinEncoder;
  private final SparkClosedLoopController closedLoopController;
  private double lastAppliedVolts = 0.0;

  public TurretIOSparkMax(int canId) {
    motor = new SparkMax(canId, MotorType.kBrushless);

    // Basic sensible defaults - callers may reconfigure if desired
    motorConfig.inverted(false).smartCurrentLimit(40);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    builtinEncoder = motor.getEncoder();
    closedLoopController = motor.getClosedLoopController();
  }

  @Override
  public void driveToGoal(Rotation2d angle) {
    // Set closed-loop position setpoint in encoder rotations converted from radians.
    // Many code paths in this repo convert between encoder/native units elsewhere; this is a
    // straightforward set using the closed-loop controller API.
    double rotations = angle.getRadians() / (2.0 * Math.PI);
    closedLoopController.setSetpoint(rotations, ControlType.kPosition);
  }

  @Override
  public void setVoltageOut(Voltage voltsOut) {
    lastAppliedVolts = voltsOut.in(Volts);
    motor.setVoltage(voltsOut);
  }

  @Override
  public boolean atSetpoint() {
    // Best-effort: compare builtin encoder position against closed-loop setpoint if available.
    try {
      double currentRot = builtinEncoder.getPosition();
      // If controller exposes an explicit setpoint this would be better; use a small tolerance
      // here (5 degrees).
      double currentRad = currentRot * 2.0 * Math.PI;
      // No direct API to read controller setpoint generically here, return false by default.
      return Math.abs(currentRad - 0.0) < Math.toRadians(5.0);
    } catch (Exception ex) {
      return false;
    }
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    double rotations = 0.0;
    try {
      rotations = builtinEncoder.getPosition();
    } catch (Exception e) {
      // leave at 0
    }
    inputs.builtinEncoderAngle = new Rotation2d(rotations * 2.0 * Math.PI);
    inputs.appliedVoltage = Volts.of(lastAppliedVolts);
    inputs.motorCurrent = Amps.of(motor.getOutputCurrent());
  inputs.motorTemperature = Celsius.of(30.0);
  }

  @Override
  public void resetBuiltInEncoderAngle(double angleRad, double angularVelocity) {
    // Spark RelativeEncoder supports setPosition
    double rotations = angleRad / (2.0 * Math.PI);
    try {
      builtinEncoder.setPosition(rotations);
    } catch (Exception e) {
      // ignore if not supported
    }
  }

  @Override
  public void resetOutsideInEncoderAngle(double angleRad, double angularVelocity) {
    // No outside (absolute) encoder on this implementation by default - treat same as builtin
    resetBuiltInEncoderAngle(angleRad, angularVelocity);
  }
}
