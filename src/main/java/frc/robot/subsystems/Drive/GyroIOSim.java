// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Radians;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.Timer;

/** Add your docs here. */
public class GyroIOSim implements GyroIO {
  private Rotation2d yaw = new Rotation2d(Degrees.of(0));
  private double lastTimestamp = Timer.getTimestamp();

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.yawPosition = yaw;
  }

  public void updateFromChassisSpeeds(ChassisVelocities speeds) {
    double timestamp = Timer.getTimestamp();
    double dt = Math.max(0.0, timestamp - lastTimestamp);
    lastTimestamp = timestamp;
    yaw = yaw.plus(new Rotation2d(Radians.of(speeds.omega * dt)));
  }
}
