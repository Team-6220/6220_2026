package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.subsystems.Drive.SwerveConstants;

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
    swerveDriveFXConfig.Feedback.SensorToMechanismRatio = SwerveConstants.DRIVE_GEAR_RATIO;

    swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable =
        SwerveConstants.driveEnableCurrentLimit;
    swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.driveMaxCurrent;
    swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerLimit = SwerveConstants.driveCurrentLimit;
    swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerTime = SwerveConstants.driveMaxCurrentTime;
    swerveDriveFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    swerveDriveFXConfig.CurrentLimits.StatorCurrentLimit = SwerveConstants.driveStatorCurrentLimit;

    /* PID Config */
    swerveDriveFXConfig.Slot0.kP = SwerveConstants.DRIVE_KP;
    swerveDriveFXConfig.Slot0.kI = SwerveConstants.DRIVE_KI;
    swerveDriveFXConfig.Slot0.kD = SwerveConstants.DRIVE_KD;

    /* Open and Closed Loop Ramping */
    swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SwerveConstants.OPEN_LOOP_RAMP;
    swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = SwerveConstants.OPEN_LOOP_RAMP;

    swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod =
        SwerveConstants.CLOSED_LOOP_RAMP;
    swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        SwerveConstants.CLOSED_LOOP_RAMP;
  }
}
