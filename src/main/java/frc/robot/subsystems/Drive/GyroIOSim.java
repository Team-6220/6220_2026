// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class GyroIOSim implements GyroIO {
  private Rotation2d yaw = new Rotation2d(Degrees.of(0));

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.yawPosition = yaw;
  }
}
