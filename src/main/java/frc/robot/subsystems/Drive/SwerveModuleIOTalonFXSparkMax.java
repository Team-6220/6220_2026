// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degree;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.RevConfigs;
import frc.robot.Robot;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Voltage;

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
    CANPort swerveCANPort = CANPort.CAN_S0;
    CANBus swerveCANBus = new CANBus(swerveCANPort);
    driveMotor = new TalonFX(swerveConfig.driveMotorID, swerveCANBus);
    driveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
    driveMotor.getConfigurator().setPosition(Degree.of(0));

    angleMotor = new SparkMax(swerveCANPort, swerveConfig.angleMotorID, MotorType.kBrushless);

    angleMotorConfig
        .inverted(SwerveConstants.angleMotorInvert)
        .idleMode(SwerveConstants.angleNeutralMode)
        .smartCurrentLimit(SwerveConstants.angleCurrentLimit);
    angleMotorConfig
        .closedLoop
        .pid(SwerveConstants.ANGLE_KP, SwerveConstants.ANGLE_KI, SwerveConstants.ANGLE_KD)
        .positionWrappingEnabled(true);
    // NOTE: use firmware instead?
    // .positionWrappingMinInput(RevConfigs.CANCoderAngleToNeoEncoder(-0.5))
    // .positionWrappingMaxInput(RevConfigs.CANCoderAngleToNeoEncoder(0.5))
    angleMotor.configure(
        angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    angleBuiltInEncoder = angleMotor.getEncoder();
    absoluteAngleEncoder = new CANcoder(swerveConfig.cancoderID, swerveCANBus);
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
                driveMotor.getPosition().getValueAsDouble(), SwerveConstants.wheelCircumference()));
    inputs.driveVelocityMps =
        MetersPerSecond.of(
            Conversions.RPSToMPS(
                driveMotor.getVelocity().getValueAsDouble(), SwerveConstants.wheelCircumference()));

    inputs.anglePositionRad =
        Radians.of(
            Radians.convertFrom(
                RevConfigs.NeoEncoderAngleToCANCoder(angleBuiltInEncoder.getPosition().get()),
                Rotations));
    inputs.angleVelocityRadPerSec =
        RadiansPerSecond.of(
            RadiansPerSecond.convertFrom(
                RevConfigs.NeoEncoderAngleToCANCoder(angleBuiltInEncoder.getVelocity().get()),
                RotationsPerSecond));

    inputs.driveAppliedVolts = Volts.of(driveMotor.getSupplyVoltage().getValueAsDouble());
    inputs.angleAppliedVolts = Volts.of(angleMotor.getBusVoltage().get());

    inputs.driveCurrentAmps = Amps.of(driveMotor.getStatorCurrent().getValueAsDouble());
    inputs.angleCurrentAmps = Amps.of(angleMotor.getOutputCurrent().get());

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
    angleController.setSetpoint(
        setpoint, com.revrobotics.spark.SparkLowLevel.ControlType.kPosition);
  }
}
