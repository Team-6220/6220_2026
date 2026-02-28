// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

/** NavX gyro IO with optional upside-down mounting correction. */
public class GyroIONavX implements GyroIO {
  private final AHRS navx = new AHRS(NavXComType.kMXP_SPI, NavXUpdateRate.k200Hz);

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // Read raw rotation from NavX
    Rotation2d raw = navx.getRotation2d();

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
