// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import frc.robot.subsystems.Drive.SwerveConfigs;

/** Add your docs here. */
public final class RobotConfig {
  public enum RobotType {
    COMP,
    PRACTICE,
    SIM
  }

  public static final RobotType TYPE = RobotType.PRACTICE;

  public static final SwerveConfig SWERVECONFIG =
      switch (TYPE) {
        case COMP -> SwerveConfigs.COMP;
        case PRACTICE -> SwerveConfigs.GEORGE;
        case SIM -> SwerveConfigs.SIM;
      };
}
