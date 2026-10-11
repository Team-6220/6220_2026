// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Meter;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radian;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import frc.robot.RevConfigs;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Voltage;

public class SwerveModuleIOSim implements SwerveModuleIO {
  private double driveVelocityMps;
  private double drivePositionMeters;
  private double angleRotations;
  private double angleTargetRotations;
  private double lastTimestamp = Timer.getTimestamp();

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    double timestamp = Timer.getTimestamp();
    double dt = Math.max(0.0, timestamp - lastTimestamp);
    lastTimestamp = timestamp;

    drivePositionMeters += driveVelocityMps * dt;
    angleRotations = angleTargetRotations;

    inputs.drivePositionMeters = Meter.of(drivePositionMeters);
    inputs.driveVelocityMps = MetersPerSecond.of(driveVelocityMps);
    inputs.anglePositionRad = Radian.of(angleRotations * 2.0 * Math.PI);
    inputs.angleVelocityRadPerSec = RadiansPerSecond.of(0.0);
    inputs.driveAppliedVolts = Volts.of(12.0 * driveVelocityMps / SwerveConstants.maxSpeed());
    inputs.angleAppliedVolts = Volts.of(0.0);
    inputs.driveCurrentAmps = Amps.of(Math.abs(driveVelocityMps) * 2.0);
    inputs.angleCurrentAmps = Amps.of(0.0);
    inputs.absoluteAngle = new Rotation2d(Rotations.of(angleRotations));
  }

  @Override
  public void setDriveControlDutyCycle(DutyCycleOut driveControl) {
    driveVelocityMps = driveControl.Output * SwerveConstants.maxSpeed();
  }

  @Override
  public void setDriveControlVelocity(VelocityVoltage driveControl) {
    driveVelocityMps = driveControl.Velocity * SwerveConstants.wheelCircumference();
  }

  @Override
  public void setDriveVoltage(Voltage volts) {
    driveVelocityMps = volts.baseUnitMagnitude() / 12.0 * SwerveConstants.maxSpeed();
  }

  @Override
  public void setAnglePosition(double setpoint) {
    angleTargetRotations = RevConfigs.NeoEncoderAngleToCANCoder(setpoint);
  }

  @Override
  public void resetToAbsolute(Rotation2d offset) {
    angleRotations = 0.0;
    angleTargetRotations = 0.0;
  }
}
