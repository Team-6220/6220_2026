package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.config.RobotConfig;

// import com.ctre.phoenix6.signals.InvertedValue;

// import frc.robot.Constants.WristConstants;;

public final class CTREConfigs {
  // public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
  public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
  public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

  public CTREConfigs() {
    /** Swerve CANCoder Configuration */
    swerveCANcoderConfig.MagnetSensor.SensorDirection = SwerveConstants.cancoderInvert;

    /** Swerve Drive Motor Configuration */
    /* Motor Inverts and Neutral Mode */
    swerveDriveFXConfig.MotorOutput.Inverted = SwerveConstants.driveMotorInvert;
    swerveDriveFXConfig.MotorOutput.NeutralMode = SwerveConstants.driveNeutralMode;

    /* Gear Ratio Config */
    swerveDriveFXConfig.Feedback.SensorToMechanismRatio = RobotConfig.SWERVECONFIG.driveGearRatio();

    swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable =
        SwerveConstants.driveEnableCurrentLimit;
    swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.driveMaxCurrent;
    swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerLimit = SwerveConstants.driveCurrentLimit;
    swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerTime = SwerveConstants.driveMaxCurrentTime;

    /* PID Config */
    swerveDriveFXConfig.Slot0.kP = RobotConfig.SWERVECONFIG.driveKP();
    swerveDriveFXConfig.Slot0.kI = RobotConfig.SWERVECONFIG.driveKI();
    swerveDriveFXConfig.Slot0.kD = RobotConfig.SWERVECONFIG.driveKD();

    /* Open and Closed Loop Ramping */
    swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod =
        RobotConfig.SWERVECONFIG.openLoopRamp();
    swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod =
        RobotConfig.SWERVECONFIG.openLoopRamp();

    swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod =
        RobotConfig.SWERVECONFIG.closedLoopRamp();
    swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        RobotConfig.SWERVECONFIG.closedLoopRamp();
  }
}
