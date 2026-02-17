package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.config.RobotConfig;

/**
 * SwerveConstants contains robot-independent swerve constants and also per-robot swerve
 * configurations (COMP/GEORGE/SIM). The public SwerveConfig instances are used by the rest of the
 * codebase.
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

  // -- Begin per-robot swerve configuration values (migrated from SwerveConfigs)

  // Private per-robot values for COMP (competition bot)
  private static final Distance COMP_WHEEL_DIAMETER = Inches.of(4.0);
  private static final double COMP_DRIVE_GEAR_RATIO = 6.75;
  private static final double COMP_ANGLE_GEAR_RATIO = (150.0 / 7.0);
  private static final double COMP_MOTOR_FREE_SPEED_RPM = 6000.0; // Kraken
  private static final Distance COMP_TRACK_WIDTH = Inches.of(22.75);
  private static final Distance COMP_WHEEL_BASE = Inches.of(20.75);
  private static final SwerveModuleConstants COMP_BACK_RIGHT_MODULE =
      new SwerveModuleConstants(
          8, 10, 4, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(-58.447));
  private static final SwerveModuleConstants COMP_BACK_LEFT_MODULE =
      new SwerveModuleConstants(
          5, 12, 2, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(-94.5));
  private static final SwerveModuleConstants COMP_FRONT_RIGHT_MODULE =
      new SwerveModuleConstants(7, 11, 1, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(23.8));
  private static final SwerveModuleConstants COMP_FRONT_LEFT_MODULE =
      new SwerveModuleConstants(6, 9, 3, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(90.5));
  private static final DCMotor COMP_DC_MOTOR = new DCMotor(12, 7.09, 366, 2, 628.32, 1);
  private static final double COMP_DRIVE_KP = 0.12;
  private static final double COMP_DRIVE_KI = 0;
  private static final double COMP_DRIVE_KD = 0;
  private static final double COMP_DRIVE_KS = 0.32;
  private static final double COMP_DRIVE_KV = 1.51;
  private static final double COMP_DRIVE_KA = 0.27;
  private static final double COMP_ANGLE_KP = 0.5;
  private static final double COMP_ANGLE_KI = 0;
  private static final double COMP_ANGLE_KD = 0.15;
  private static final double COMP_OPEN_LOOP_RAMP = 0.25;
  private static final double COMP_CLOSED_LOOP_RAMP = 0;

  // Private per-robot values for GEORGE (practice bot)
  private static final Distance GEORGE_WHEEL_DIAMETER = Inches.of(4.0);
  private static final double GEORGE_DRIVE_GEAR_RATIO = 6.75;
  private static final double GEORGE_ANGLE_GEAR_RATIO = (150.0 / 7.0);
  private static final double GEORGE_MOTOR_FREE_SPEED_RPM = 6380.0; // Falcon
  private static final Distance GEORGE_TRACK_WIDTH = Inches.of(22.75);
  private static final Distance GEORGE_WHEEL_BASE = Inches.of(22.75);
  private static final SwerveModuleConstants GEORGE_BACK_RIGHT_MODULE =
      new SwerveModuleConstants(
          1, 2, 3, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(-41.22));
  private static final SwerveModuleConstants GEORGE_BACK_LEFT_MODULE =
      new SwerveModuleConstants(
          4, 5, 6, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(12.563));
  private static final SwerveModuleConstants GEORGE_FRONT_RIGHT_MODULE =
      new SwerveModuleConstants(
          7, 8, 9, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(-85.869));
  private static final SwerveModuleConstants GEORGE_FRONT_LEFT_MODULE =
      new SwerveModuleConstants(
          10, 11, 12, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(-59.386 - 90));
  private static final DCMotor GEORGE_DC_MOTOR = new DCMotor(12, 4.69, 257, 1.5, 668.112, 1);
  private static final double GEORGE_DRIVE_KP = 0.12;
  private static final double GEORGE_DRIVE_KI = 0;
  private static final double GEORGE_DRIVE_KD = 0;
  private static final double GEORGE_DRIVE_KS = 0.32;
  private static final double GEORGE_DRIVE_KV = 1.51;
  private static final double GEORGE_DRIVE_KA = 0.27;
  private static final double GEORGE_ANGLE_KP = 0.5;
  private static final double GEORGE_ANGLE_KI = 0;
  private static final double GEORGE_ANGLE_KD = 0.15;
  private static final double GEORGE_OPEN_LOOP_RAMP = 0.25;
  private static final double GEORGE_CLOSED_LOOP_RAMP = 0;

  // -- End per-robot swerve configuration values

  // Public, robot-selected primitive constants (preferred pattern requested)
  public static final Distance WHEEL_DIAMETER;
  public static final double DRIVE_GEAR_RATIO;
  public static final double ANGLE_GEAR_RATIO;
  public static final double MOTOR_FREE_SPEED_RPM;
  public static final Distance TRACK_WIDTH;
  public static final Distance WHEEL_BASE;
  public static final SwerveModuleConstants BACK_RIGHT_MODULE;
  public static final SwerveModuleConstants BACK_LEFT_MODULE;
  public static final SwerveModuleConstants FRONT_RIGHT_MODULE;
  public static final SwerveModuleConstants FRONT_LEFT_MODULE;
  public static final DCMotor DC_MOTOR;
  public static final double DRIVE_KP;
  public static final double DRIVE_KI;
  public static final double DRIVE_KD;
  public static final double DRIVE_KS;
  public static final double DRIVE_KV;
  public static final double DRIVE_KA;
  public static final double ANGLE_KP;
  public static final double ANGLE_KI;
  public static final double ANGLE_KD;
  public static final double OPEN_LOOP_RAMP;
  public static final double CLOSED_LOOP_RAMP;

  static {
    switch (RobotConfig.TYPE) {
      case COMP -> {
        WHEEL_DIAMETER = COMP_WHEEL_DIAMETER;
        DRIVE_GEAR_RATIO = COMP_DRIVE_GEAR_RATIO;
        ANGLE_GEAR_RATIO = COMP_ANGLE_GEAR_RATIO;
        MOTOR_FREE_SPEED_RPM = COMP_MOTOR_FREE_SPEED_RPM;
        TRACK_WIDTH = COMP_TRACK_WIDTH;
        WHEEL_BASE = COMP_WHEEL_BASE;
        BACK_RIGHT_MODULE = COMP_BACK_RIGHT_MODULE;
        BACK_LEFT_MODULE = COMP_BACK_LEFT_MODULE;
        FRONT_RIGHT_MODULE = COMP_FRONT_RIGHT_MODULE;
        FRONT_LEFT_MODULE = COMP_FRONT_LEFT_MODULE;
        DC_MOTOR = COMP_DC_MOTOR;
        DRIVE_KP = COMP_DRIVE_KP;
        DRIVE_KI = COMP_DRIVE_KI;
        DRIVE_KD = COMP_DRIVE_KD;
        DRIVE_KS = COMP_DRIVE_KS;
        DRIVE_KV = COMP_DRIVE_KV;
        DRIVE_KA = COMP_DRIVE_KA;
        ANGLE_KP = COMP_ANGLE_KP;
        ANGLE_KI = COMP_ANGLE_KI;
        ANGLE_KD = COMP_ANGLE_KD;
        OPEN_LOOP_RAMP = COMP_OPEN_LOOP_RAMP;
        CLOSED_LOOP_RAMP = COMP_CLOSED_LOOP_RAMP;
      }
      case PRACTICE -> {
        WHEEL_DIAMETER = GEORGE_WHEEL_DIAMETER;
        DRIVE_GEAR_RATIO = GEORGE_DRIVE_GEAR_RATIO;
        ANGLE_GEAR_RATIO = GEORGE_ANGLE_GEAR_RATIO;
        MOTOR_FREE_SPEED_RPM = GEORGE_MOTOR_FREE_SPEED_RPM;
        TRACK_WIDTH = GEORGE_TRACK_WIDTH;
        WHEEL_BASE = GEORGE_WHEEL_BASE;
        BACK_RIGHT_MODULE = GEORGE_BACK_RIGHT_MODULE;
        BACK_LEFT_MODULE = GEORGE_BACK_LEFT_MODULE;
        FRONT_RIGHT_MODULE = GEORGE_FRONT_RIGHT_MODULE;
        FRONT_LEFT_MODULE = GEORGE_FRONT_LEFT_MODULE;
        DC_MOTOR = GEORGE_DC_MOTOR;
        DRIVE_KP = GEORGE_DRIVE_KP;
        DRIVE_KI = GEORGE_DRIVE_KI;
        DRIVE_KD = GEORGE_DRIVE_KD;
        DRIVE_KS = GEORGE_DRIVE_KS;
        DRIVE_KV = GEORGE_DRIVE_KV;
        DRIVE_KA = GEORGE_DRIVE_KA;
        ANGLE_KP = GEORGE_ANGLE_KP;
        ANGLE_KI = GEORGE_ANGLE_KI;
        ANGLE_KD = GEORGE_ANGLE_KD;
        OPEN_LOOP_RAMP = GEORGE_OPEN_LOOP_RAMP;
        CLOSED_LOOP_RAMP = GEORGE_CLOSED_LOOP_RAMP;
      }
      case SIM -> {
        WHEEL_DIAMETER = COMP_WHEEL_DIAMETER;
        DRIVE_GEAR_RATIO = COMP_DRIVE_GEAR_RATIO;
        ANGLE_GEAR_RATIO = COMP_ANGLE_GEAR_RATIO;
        MOTOR_FREE_SPEED_RPM = COMP_MOTOR_FREE_SPEED_RPM;
        TRACK_WIDTH = COMP_TRACK_WIDTH;
        WHEEL_BASE = COMP_WHEEL_BASE;
        BACK_RIGHT_MODULE = COMP_BACK_RIGHT_MODULE;
        BACK_LEFT_MODULE = COMP_BACK_LEFT_MODULE;
        FRONT_RIGHT_MODULE = COMP_FRONT_RIGHT_MODULE;
        FRONT_LEFT_MODULE = COMP_FRONT_LEFT_MODULE;
        DC_MOTOR = COMP_DC_MOTOR;
        DRIVE_KP = COMP_DRIVE_KP;
        DRIVE_KI = COMP_DRIVE_KI;
        DRIVE_KD = COMP_DRIVE_KD;
        DRIVE_KS = COMP_DRIVE_KS;
        DRIVE_KV = COMP_DRIVE_KV;
        DRIVE_KA = COMP_DRIVE_KA;
        ANGLE_KP = COMP_ANGLE_KP;
        ANGLE_KI = COMP_ANGLE_KI;
        ANGLE_KD = COMP_ANGLE_KD;
        OPEN_LOOP_RAMP = COMP_OPEN_LOOP_RAMP;
        CLOSED_LOOP_RAMP = COMP_CLOSED_LOOP_RAMP;
      }
      default -> throw new IllegalStateException("Unknown RobotType: " + RobotConfig.TYPE);
    }
  }

  // Convenience helpers that mirror what SwerveConfig provided so callers can keep using
  // RobotConfig.SWERVECONFIG.* if needed while you migrate code to the primitive constants.
  public static double wheelCircumference() {
    return WHEEL_DIAMETER.in(Meters) * Math.PI;
  }

  public static double wheelRadius() {
    return WHEEL_DIAMETER.in(Meters) / 2.0;
  }

  public static SwerveDriveKinematics kinematics() {
    double tw = TRACK_WIDTH.in(Meters);
    double wb = WHEEL_BASE.in(Meters);
    return new SwerveDriveKinematics(
        new Translation2d(wb / 2.0, tw / 2.0),
        new Translation2d(wb / 2.0, -tw / 2.0),
        new Translation2d(-wb / 2.0, tw / 2.0),
        new Translation2d(-wb / 2.0, -tw / 2.0));
  }

  /** Meters per second */
  public static double maxSpeed() {
    return 0.80
        * (MOTOR_FREE_SPEED_RPM / 60.0)
        * wheelCircumference()
        * DRIVE_GEAR_RATIO; // assume 80% efficiency in real life
  }

  /** Radians per second */
  public static double maxAngularVelocity() {
    double tw = TRACK_WIDTH.in(Meters);
    double wb = WHEEL_BASE.in(Meters);
    return maxSpeed() / Math.hypot(tw / 2.0, wb / 2.0);
  }

  public static ModuleConfig swerveModuleConfig() {
    return new ModuleConfig(
        wheelRadius(), maxSpeed(), 1.0, DC_MOTOR, SwerveConstants.driveCurrentLimit, 1);
  }
}
