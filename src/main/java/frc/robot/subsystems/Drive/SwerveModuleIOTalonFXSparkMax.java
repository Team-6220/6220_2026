// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.RevConfigs;
import frc.robot.Robot;
import frc.robot.SwerveConstants;
import frc.robot.config.RobotConfig;

/** TalonFX Drive Motor, SparkMax Turn Motor */
public class SwerveModuleIOTalonFXSparkMax implements SwerveModuleIO {
  private final TalonFX driveMotor;
  private final SparkMax angleMotor;
  private SparkMaxConfig angleMotorConfig = new SparkMaxConfig();
  private final RelativeEncoder angleBuiltInEncoder;
  private final CANcoder absoluteAngleEncoder;
  private final Rotation2d angleOffset;
  private final SparkClosedLoopController angleController;

  public SwerveModuleIOTalonFXSparkMax(SwerveModuleConstants swerveConfig) {
    driveMotor = new TalonFX(swerveConfig.driveMotorID);
    driveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
    driveMotor.getConfigurator().setPosition(Degree.of(0));

    angleMotor = new SparkMax(swerveConfig.angleMotorID, MotorType.kBrushless);

    angleMotorConfig
        .inverted(SwerveConstants.angleMotorInvert)
        .idleMode(SwerveConstants.angleNeutralMode)
        .smartCurrentLimit(SwerveConstants.angleCurrentLimit);
    angleMotorConfig
        .closedLoop
        .pid(
            RobotConfig.SWERVECONFIG.angleKP(),
            RobotConfig.SWERVECONFIG.angleKI(),
            RobotConfig.SWERVECONFIG.angleKD())
        .positionWrappingEnabled(true)
        .positionWrappingMinInput(0)
        .positionWrappingMaxInput(RevConfigs.CANCoderAngleToNeoEncoder(1));
    angleMotor.configure(
        angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    angleBuiltInEncoder = angleMotor.getEncoder();
    absoluteAngleEncoder = new CANcoder(swerveConfig.cancoderID);
    absoluteAngleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

    angleController = angleMotor.getClosedLoopController();
    angleOffset = swerveConfig.angleOffset;
    resetToAbsolute(angleOffset);
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    inputs.drivePositionMeters =
        Meters.of(
            Conversions.rotationsToMeters(
                driveMotor.getPosition().getValueAsDouble(),
                RobotConfig.SWERVECONFIG.wheelCircumference()));
    inputs.driveVelocityMps =
        MetersPerSecond.of(
            Conversions.RPSToMPS(
                driveMotor.getVelocity().getValueAsDouble(),
                RobotConfig.SWERVECONFIG.wheelCircumference()));

    inputs.anglePositionRad =
        Radians.of(
            Radians.convertFrom(
                RevConfigs.NeoEncoderAngleToCANCoder(angleBuiltInEncoder.getPosition()),
                Rotations));
    inputs.angleVelocityRadPerSec =
        RadiansPerSecond.of(
            RadiansPerSecond.convertFrom(
                RevConfigs.NeoEncoderAngleToCANCoder(angleBuiltInEncoder.getVelocity()),
                RotationsPerSecond));

    inputs.driveAppliedVolts = Volts.of(driveMotor.getSupplyVoltage().getValueAsDouble());
    inputs.angleAppliedVolts = Volts.of(angleMotor.getBusVoltage());

    inputs.driveCurrentAmps = Amps.of(driveMotor.getStatorCurrent().getValueAsDouble());
    inputs.angleCurrentAmps = Amps.of(angleMotor.getOutputCurrent());

    inputs.absoluteAngle =
        new Rotation2d(Rotations.of(absoluteAngleEncoder.getAbsolutePosition().getValueAsDouble()));
  }

  @Override
  public void resetToAbsolute(Rotation2d offset) {
    double absolutePosition =
        absoluteAngleEncoder.getAbsolutePosition().getValueAsDouble() - offset.getRotations();
    angleBuiltInEncoder.setPosition(RevConfigs.CANCoderAngleToNeoEncoder(absolutePosition));
  }

  @Override
  public void setDriveVoltage(Voltage volts) {
    driveMotor.setVoltage(volts.baseUnitMagnitude());
  }

  @Override
  public void setAngleVoltage(Voltage volts) {
    angleMotor.setVoltage(volts);
  }

  @Override
  public void setDriveControlVelocity(VelocityVoltage driveControl) {
    driveMotor.setControl(driveControl);
  }

  @Override
  public void setDriveControlDutyCycle(DutyCycleOut driveControl) {
    driveMotor.setControl(driveControl);
  }

  @Override
  public void setAnglePosition(double setpoint) {
    angleController.setSetpoint(setpoint, ControlType.kPosition);
  }
}
