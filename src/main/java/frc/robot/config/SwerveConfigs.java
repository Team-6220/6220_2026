// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.config.ModuleConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.SwerveConstants;
public final class SwerveConfigs {
    /**Place holder for sewrve config */
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

    DCMotor dcMotor
) {

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
            new Translation2d( wb / 2.0, tw / 2.0),
            new Translation2d( wb / 2.0, -tw / 2.0),
            new Translation2d(-wb / 2.0, tw / 2.0),
            new Translation2d(-wb / 2.0, -tw / 2.0)
        );
    }
    /** Meters per second */
    public double maxSpeed() {
        return 0.80 * (motorFreeSpeedRPM / 60.0) * wheelCircumference() * driveGearRatio; //assume 80% efficiency in real life
    }
    /** Radians per second */
    public double maxAngularVelocity() {
        double tw = trackWidth.in(Meters);
        double wb = wheelBase.in(Meters);
        return maxSpeed() / Math.hypot(tw / 2.0, wb / 2.0);
    }

    public ModuleConfig swerveModuleConfig()
    {
        return new ModuleConfig(RobotConfig.SWERVECONFIG.wheelRadius(), RobotConfig.SWERVECONFIG.maxSpeed(), 1.0, dcMotor, SwerveConstants.driveCurrentLimit, 1);
    }
      
}

    public static final SwerveConfig COMP = new SwerveConfig(
        Inches.of(4.0),
        6.75,
        (150.0 / 7.0),
        6000.0, // motorFreeSpeedRPM for Kraken

        Inches.of(22.75),
        Inches.of(20.75),

        new SwerveModuleConstants(8, 10, 4, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(-58.447)),
        new SwerveModuleConstants(5, 12, 2, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(-94.5)),
        new SwerveModuleConstants(7, 11, 1, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(23.8)),
        new SwerveModuleConstants(6, 9, 3, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(90.5)),

        // https://docs.wcproducts.com/welcome/electronics/kraken-x60/kraken-x60-motor/overview-and-features/physical-specifications
        new DCMotor(12, 7.09, 366, 2, 628.32, 1)
    );

    public static final SwerveConfig PRACTICE = new SwerveConfig(
        Inches.of(4.0),
        6.75,
        (150.0 / 7.0),
        6380.0, // motorFreeSpeedRPM for Falcon

        Inches.of(22.75),
        Inches.of(22.75),

        new SwerveModuleConstants(8, 10, 4, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(-58.447)),
        new SwerveModuleConstants(5, 12, 2, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(-94.5)),
        new SwerveModuleConstants(7, 11, 1, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(23.8)),
        new SwerveModuleConstants(6, 9, 3, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(90.5)),
        
        // https://store.ctr-electronics.com/products/falcon-500-powered-by-talon-fx
        new DCMotor(12, 4.69, 257, 1.5, 668.112, 1)
    );

    public static final SwerveConfig SIM = COMP;


    private SwerveConfigs() {}

}
