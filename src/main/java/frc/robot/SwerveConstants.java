package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class SwerveConstants {

  public static int swerveAlignUpdateSecond = 20;

  public static final int pigeonID = 1;

  public static final double translation_kP = 2.518;
  public static final double translation_kI = 0.6;
  public static final double translation_kD = 0.0;
  public static final double rotation_kP = 1.35;
  public static final double rotation_kI = 1.25;
  public static final double rotation_kD = 0.0;

  public static final COTSTalonFXSwerveConstants chosenModule =
      COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(
          COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

  /* Drivetrain Constants */
  public static final double trackWidth =
      Units.inchesToMeters(21); // TODO: This must be tuned to specific robot
  public static final double wheelBase =
      Units.inchesToMeters(26); // TODO: This must be tuned to specific robot
  public static final double wheelCircumference = chosenModule.wheelCircumference;
  public static final double wheelRadius = chosenModule.wheelDiameter / 2;

  /* Swerve Kinematics
   * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
  public static final SwerveDriveKinematics swerveKinematics =
      new SwerveDriveKinematics(
          new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
          new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

  /* Module Gear Ratios */
  public static final double driveGearRatio = chosenModule.driveGearRatio;
  public static final double angleGearRatio = chosenModule.angleGearRatio;

  /* Motor Inverts */
  public static final boolean angleMotorInvert =
      chosenModule.driveMotorInvert == InvertedValue.CounterClockwise_Positive;
  public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

  /* Angle Encoder Invert */
  public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

  /* Swerve Current Limiting */
  public static final int angleCurrentLimit = 25;
  public static final int angleCurrentThreshold = 40;
  public static final double angleCurrentThresholdTime = 0.1;
  public static final boolean angleEnableCurrentLimit = true;

  public static final double driveCurrentLimit = 35;
  public static final double driveMaxCurrent = 60;
  public static final double driveMaxCurrentTime = 0.1;
  public static final boolean driveEnableCurrentLimit = true;

  /* Swerve module configs -- for pathplanner autobuilder (auto)
   * API: https://pathplanner.dev/api/java/com/pathplanner/lib/config/ModuleConfig.html
   */
  public static final DCMotor krackonX60 =
      new DCMotor(
          12, 7.09, 366, 2, 628.32,
          4); // https://docs.wcproducts.com/kraken-x60/kraken-x60-motor/overview-and-features/motor-performance
  public static final ModuleConfig swerveModuleConfig =
      new ModuleConfig(
          wheelRadius, SwerveConstants.maxSpeed, 1.0, krackonX60, driveCurrentLimit, 4);

  /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
   * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
  public static final double openLoopRamp = 0.25;
  public static final double closedLoopRamp = 0.0;

  /* Angle Motor PID Values */
  public static final double angleKP = 0.5;
  public static final double angleKI = 0;
  public static final double angleKD = 0.15;

  /* Drive Motor PID Values */
  public static final double driveKP = 0.12; // TODO: This must be tuned to specific robot
  public static final double driveKI = 0.0;
  public static final double driveKD = 0.0;
  public static final double driveKF = 0.0;

  /* Drive Motor Characterization Values From SYSID */
  public static final double driveKS = 0.32; // TODO: This must be tuned to specific robot
  public static final double driveKV = 1.51;
  public static final double driveKA = 0.27;

  // Turning Pid Constants
  public static final double turnKP = 1;
  public static final double turnKD = 0;
  public static final double turnKI = 1.7;
  public static final double turnMaxVel = 400;
  public static final double turnMaxAccel = 800;
  public static final double turnTolerance = 5;
  public static final double turnIZone = .4;

  // X + Y position Pid Constants for Vision autos
  public static final double xKP = 2.25;
  public static final double xKD = 0;
  public static final double xKI = 0;
  public static final double xMaxVel = 400;
  public static final double xMaxAccel = 800;
  public static final double xTolerance = 1.75;
  public static final double xIZone = .4;

  public static final double yKP = 2.25;
  public static final double yKD = 0;
  public static final double yKI = 0;
  public static final double yMaxVel = 400;
  public static final double yMaxAccel = 800;
  public static final double yTolerance = 1.75;
  public static final double yIZone = .4;

  /* Swerve Profiling Values */
  /** Meters per Second */
  public static final double maxSpeed = 6380.0 / 60.0 * wheelCircumference * driveGearRatio;

  /** Radians per Second */
  public static final double maxAngularVelocity =
      maxSpeed / Math.hypot(trackWidth / 2.0, wheelBase / 2.0);

  /* Neutral Modes */
  public static final IdleMode angleNeutralMode = IdleMode.kBrake;
  public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

  /* Module Specific Constants */
  // Back Right Module 0
  public static final class Mod0 { // FIXME: This must be tuned to specific robot
    public static final int driveMotorID = 8;
    public static final int angleMotorID = 10;
    public static final int canCoderID = 4;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-58.447);
    public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
  }

  // Back Left Module 1
  public static final class Mod1 { // FIXME: This must be tuned to specific robot
    public static final int driveMotorID = 5;
    public static final int angleMotorID = 12;
    public static final int canCoderID = 2;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-94.5);
    public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
  }

  // Front Right - Module 2
  public static final class Mod2 { // FIXME: This must be tuned to specific robot
    public static final int driveMotorID = 7;
    public static final int angleMotorID = 11;
    public static final int canCoderID = 1;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(23.8);
    public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
  }

  // Front left Module 3
  public static final class Mod3 { // FIXME: This must be tuned to specific robot
    public static final int driveMotorID = 6;
    public static final int angleMotorID = 9;
    public static final int canCoderID = 3;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(90.5);
    public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
  }
}
