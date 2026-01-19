package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.config.RobotConfig;

public final class SwerveConstants {

  /* Motor Inverts */
  public static final boolean angleMotorInvert = true;
  public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;

  /* Angle Encoder Invert */
  public static final SensorDirectionValue cancoderInvert =
      SensorDirectionValue.CounterClockwise_Positive;

  /* Swerve Current Limiting */
  public static final int angleCurrentLimit = 25;
  public static final int angleCurrentThreshold = 40;
  public static final double angleCurrentThresholdTime = 0.1;
  public static final boolean angleEnableCurrentLimit = true;

  public static final double driveCurrentLimit = 35;
  public static final double driveMaxCurrent = 60;
  public static final double driveMaxCurrentTime = 0.1;
  public static final boolean driveEnableCurrentLimit = true;

  /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
   * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
  public static final double openLoopRamp = 0.25;
  public static final double closedLoopRamp = 0.0;

  /* Drive Motor PID Values */
  public static final double driveKP = 0.12; // TODO: This must be tuned to specific robot
  public static final double driveKI = 0.0;
  public static final double driveKD = 0.0;

  /* Drive Motor Characterization Values*/
  public static final double driveKS = 0.32; // TODO: This must be tuned to specific robot
  public static final double driveKV = 1.51;
  public static final double driveKA = 0.27;

  /* Angle Motor PID Values */
  public static final double angleKP = 0.5;
  public static final double angleKI = 0;
  public static final double angleKD = 0.15;

  /* Neutral Modes */
  public static final IdleMode angleNeutralMode = IdleMode.kBrake;
  public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

  // DC motor and swerve
  /* Swerve module configs -- for pathplanner autobuilder (auto)
   * API: https://pathplanner.dev/api/java/com/pathplanner/lib/config/ModuleConfig.html
   */
  public static final DCMotor krackonX60 =
      new DCMotor(
          12, 7.09, 366, 2, 628.32,
          4); // https://docs.wcproducts.com/kraken-x60/kraken-x60-motor/overview-and-features/motor-performance
  public static final ModuleConfig swerveModuleConfig =
      new ModuleConfig(
          RobotConfig.SWERVECONFIG.wheelRadius(),
          RobotConfig.SWERVECONFIG.maxSpeed(),
          1.0,
          krackonX60,
          driveCurrentLimit,
          4);
  /* Module Gear Ratios */
  public static final double driveGearRatio = RobotConfig.SWERVECONFIG.driveGearRatio();
  public static final double angleGearRatio = RobotConfig.SWERVECONFIG.angleGearRatio();
}
