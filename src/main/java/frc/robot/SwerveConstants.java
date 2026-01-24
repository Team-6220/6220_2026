package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * These are constants that are considered robot-independent; therefore they are not put into the
 * configs
 */
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

  /* Neutral Modes */
  public static final IdleMode angleNeutralMode = IdleMode.kBrake;
  public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;
}
