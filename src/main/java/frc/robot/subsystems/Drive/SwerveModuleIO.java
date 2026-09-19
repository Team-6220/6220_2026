// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import static org.wpilib.units.Units.Amp;
import static org.wpilib.units.Units.Meter;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radian;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Voltage;

/** Add your docs here. */
public interface SwerveModuleIO {
  public static class SwerveModuleIOInputs {
    public Distance drivePositionMeters = Meter.of(0);
    public LinearVelocity driveVelocityMps = MetersPerSecond.of(0);

    public Angle anglePositionRad = Radian.of(0.0);
    public AngularVelocity angleVelocityRadPerSec = RadiansPerSecond.of(0);

    public Voltage driveAppliedVolts = Volts.of(0);
    public Voltage angleAppliedVolts = Volts.of(0);

    public Current driveCurrentAmps = Amp.of(0);
    public Current angleCurrentAmps = Amp.of(0);

    public Rotation2d absoluteAngle = new Rotation2d();
  }

  default void updateInputs(SwerveModuleIOInputs inputs) {}

  default void setDriveVoltage(Voltage volts) {}

  default void setDriveControlVelocity(VelocityVoltage driveControl) {}

  default void setDriveControlDutyCycle(DutyCycleOut driveControl) {}

  default void setAngleVoltage(Voltage volts) {}

  default void setAnglePosition(double setpoint) {}

  default void resetToAbsolute(Rotation2d offset) {}
}
