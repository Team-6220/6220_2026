// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.config.SwerveConfig;

public final class SwerveConfigs {
  public static final SwerveConfig COMP =
      new SwerveConfig(
          Inches.of(4.0),
          6.75,
          (150.0 / 7.0),
          6000.0, // motorFreeSpeedRPM for Kraken
          Inches.of(22.75),
          Inches.of(20.75),
          new SwerveModuleConstants(
              8, 10, 4, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(-58.447)),
          new SwerveModuleConstants(
              5, 12, 2, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(-94.5)),
          new SwerveModuleConstants(
              7, 11, 1, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(23.8)),
          new SwerveModuleConstants(
              6, 9, 3, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(90.5)),

          // https://docs.wcproducts.com/welcome/electronics/kraken-x60/kraken-x60-motor/overview-and-features/physical-specifications
          new DCMotor(12, 7.09, 366, 2, 628.32, 1),
          0.12,
          0,
          0,
          0.32,
          1.51,
          0.27,
          0.5,
          0,
          0.15,
          0.25,
          0);

  public static final SwerveConfig GEORGE =
      new SwerveConfig(
          Inches.of(4.0),
          6.75,
          (150.0 / 7.0),
          6380.0, // motorFreeSpeedRPM for Falcon
          Inches.of(22.75),
          Inches.of(22.75),
          new SwerveModuleConstants(
              1, 2, 3, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(-41.22)),
          new SwerveModuleConstants(
              4, 5, 6, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(12.563)),
          new SwerveModuleConstants(
              7, 8, 9, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(-85.869)),
          new SwerveModuleConstants(
              10, 11, 12, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(-59.386)),

          // https://store.ctr-electronics.com/products/falcon-500-powered-by-talon-fx
          new DCMotor(12, 4.69, 257, 1.5, 668.112, 1),
          0.12,
          0,
          0,
          0.32,
          1.51,
          0.27,
          0.5,
          0,
          0.15,
          0.25,
          0);

  public static final SwerveConfig SIM = COMP;

  private SwerveConfigs() {}
}
