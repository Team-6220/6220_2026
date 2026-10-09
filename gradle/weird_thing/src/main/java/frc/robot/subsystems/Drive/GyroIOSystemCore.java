// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import frc.robot.Constants;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.hardware.imu.OnboardIMU.MountOrientation;
import org.wpilib.math.geometry.Rotation2d;

/** Add your docs here. */
public class GyroIOSystemCore implements GyroIO {
  private final OnboardIMU builtinIMU = new OnboardIMU(MountOrientation.FLAT);

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    Rotation2d raw = builtinIMU.getRotation2d();

    if (Constants.GYRO_UPSIDEDOWN) {
      // If the NavX is mounted upside-down on the robot, its reported yaw will be
      // effectively rotated and inverted. Apply a simple correction here so users
      // can flip a single constant in `Constants` instead of changing wiring.
      //
      // This correction mirrors what we observe on upside-down mounts: negate the
      // angle sign and add 180 degrees to re-align to the robot frame.
      double correctedDeg = -raw.getDegrees() + 180.0;
      inputs.yawPosition = Rotation2d.fromDegrees(correctedDeg);
    } else {
      inputs.yawPosition = raw;
    }
  }
}
