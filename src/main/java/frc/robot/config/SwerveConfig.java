// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.config.ModuleConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.SwerveConstants;

/** Place holder for swerve configurations */
public record SwerveConfig(
    Distance wheelDiameter,
    double driveGearRatio,
    double angleGearRatio,
    double motorFreeSpeedRPM,
    Distance trackWidth,
    Distance wheelBase,
    SwerveModuleConstants backRightMod0,
    SwerveModuleConstants backLeftMod1,
    SwerveModuleConstants frontRightMod2,
    SwerveModuleConstants frontLeftMod3,
    DCMotor dcMotor,
    double driveKP,
    double driveKI,
    double driveKD,
    double driveKS,
    double driveKV,
    double driveKA,
    double angleKP,
    double angleKI,
    double angleKD,
    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    double openLoopRamp,
    double closedLoopRamp) {

  public double wheelCircumference() {
    return wheelDiameter.in(Meters) * Math.PI;
  }

  public double wheelRadius() {
    return wheelDiameter.in(Meters) / 2;
  }

  public SwerveDriveKinematics kinematics() {
    double tw = trackWidth.in(Meters);
    double wb = wheelBase.in(Meters);
    return new SwerveDriveKinematics(
        new Translation2d(wb / 2.0, tw / 2.0),
        new Translation2d(wb / 2.0, -tw / 2.0),
        new Translation2d(-wb / 2.0, tw / 2.0),
        new Translation2d(-wb / 2.0, -tw / 2.0));
  }

  /** Meters per second */
  public double maxSpeed() {
    return 0.80
        * (motorFreeSpeedRPM / 60.0)
        * wheelCircumference()
        * driveGearRatio; // assume 80% efficiency in real life
  }

  /** Radians per second */
  public double maxAngularVelocity() {
    double tw = trackWidth.in(Meters);
    double wb = wheelBase.in(Meters);
    return maxSpeed() / Math.hypot(tw / 2.0, wb / 2.0);
  }

  public ModuleConfig swerveModuleConfig() {
    return new ModuleConfig(
        RobotConfig.SWERVECONFIG.wheelRadius(),
        RobotConfig.SWERVECONFIG.maxSpeed(),
        1.0,
        dcMotor,
        SwerveConstants.driveCurrentLimit,
        1);
  }
}
