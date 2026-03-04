// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Real hardware implementation of {@link ShooterIO} using TalonFX motor controllers. */
public class ShooterIOReal implements ShooterIO {

  // Motor CAN IDs
  private static final int MOTOR_41_ID = 41;
  private static final int MOTOR_1_ID = 1;
  private static final int MOTOR_9_ID = 9;
  private static final int MOTOR_31_ID = 31;
  private static final int MOTOR_2_ID = 2;

  // Current limit in amps
  private static final double CURRENT_LIMIT = 40.0;

  private final TalonFX m_motor41;
  private final TalonFX m_motor1;
  private final TalonFX m_motor9;
  private final TalonFX m_motor31;
  private final TalonFX m_motor2;

  private final VelocityVoltage m_velocityRequest;

  public ShooterIOReal() {
    m_motor41 = new TalonFX(MOTOR_41_ID);
    m_motor1 = new TalonFX(MOTOR_1_ID);
    m_motor9 = new TalonFX(MOTOR_9_ID);
    m_motor31 = new TalonFX(MOTOR_31_ID);
    m_motor2 = new TalonFX(MOTOR_2_ID);
    m_velocityRequest = new VelocityVoltage(0).withSlot(0);

    configureMotors();
  }

  private void configureMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Current limits
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
    config.CurrentLimits = currentLimits;

    // Motor output config
    MotorOutputConfigs outputConfig = new MotorOutputConfigs();
    outputConfig.NeutralMode = NeutralModeValue.Coast;

    // Motor 41 -> CounterClockwise Positive
    outputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput = outputConfig;
    m_motor41.getConfigurator().apply(config);

    // Motor 1 -> Clockwise Positive (flipped to test)
    outputConfig.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput = outputConfig;
    m_motor1.getConfigurator().apply(config);

    // Motors 9 and 31 spin the same way -> CounterClockwise Positive (default)
    outputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput = outputConfig;
    m_motor9.getConfigurator().apply(config);
    m_motor31.getConfigurator().apply(config);
    m_motor2.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(Inputs inputs) {
    inputs.motor41VelocityRPS = m_motor41.getVelocity().getValueAsDouble();
    inputs.motor1VelocityRPS = m_motor1.getVelocity().getValueAsDouble();
    inputs.motor9VelocityRPS = m_motor9.getVelocity().getValueAsDouble();
    inputs.motor31VelocityRPS = m_motor31.getVelocity().getValueAsDouble();
    inputs.motor2VelocityRPS = m_motor2.getVelocity().getValueAsDouble();

    inputs.motor41Voltage = m_motor41.getMotorVoltage().getValueAsDouble();
    inputs.motor1Voltage = m_motor1.getMotorVoltage().getValueAsDouble();
    inputs.motor9Voltage = m_motor9.getMotorVoltage().getValueAsDouble();
    inputs.motor31Voltage = m_motor31.getMotorVoltage().getValueAsDouble();
    inputs.motor2Voltage = m_motor2.getMotorVoltage().getValueAsDouble();

    inputs.motor41Current = m_motor41.getSupplyCurrent().getValueAsDouble();
    inputs.motor1Current = m_motor1.getSupplyCurrent().getValueAsDouble();
    inputs.motor9Current = m_motor9.getSupplyCurrent().getValueAsDouble();
    inputs.motor31Current = m_motor31.getSupplyCurrent().getValueAsDouble();
    inputs.motor2Current = m_motor2.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void setVelocityRPS(double rps) {
    m_motor41.setControl(m_velocityRequest.withVelocity(rps));
    m_motor1.setControl(m_velocityRequest.withVelocity(rps));
    m_motor9.setControl(m_velocityRequest.withVelocity(rps));
    m_motor31.setControl(m_velocityRequest.withVelocity(rps));
    m_motor2.setControl(m_velocityRequest.withVelocity(rps));
  }

  @Override
  public void stop() {
    m_motor41.stopMotor();
    m_motor1.stopMotor();
    m_motor9.stopMotor();
    m_motor31.stopMotor();
    m_motor2.stopMotor();
  }

  @Override
  public void stopTopGroup() {
    m_motor41.stopMotor();
    m_motor1.stopMotor();
  }

  @Override
  public void stopBottomGroup() {
    m_motor9.stopMotor();
    m_motor2.stopMotor();
    m_motor31.stopMotor();
  }

  @Override
  public void setPercentOutput(double percent) {
    m_motor41.set(percent);
    m_motor1.set(percent);
    m_motor9.set(percent);
    m_motor31.set(percent);
    m_motor2.set(percent);
  }

  @Override
  public void setTopGroupPercent(double percent) {
    m_motor41.set(percent);
    m_motor1.set(percent);
  }

  @Override
  public void setBottomGroupPercent(double percent) {
    m_motor9.set(percent);
    m_motor2.set(percent);
    m_motor31.set(percent);
  }

  @Override
  public void applyPIDConfigs(
      double kP, double kI, double kD, double kV, double kS, double kA) {
    Slot0Configs pidConfigs = new Slot0Configs();
    pidConfigs.kP = kP;
    pidConfigs.kI = kI;
    pidConfigs.kD = kD;
    pidConfigs.kV = kV;
    pidConfigs.kS = kS;
    pidConfigs.kA = kA;

    m_motor41.getConfigurator().apply(pidConfigs);
    m_motor1.getConfigurator().apply(pidConfigs);
    m_motor9.getConfigurator().apply(pidConfigs);
    m_motor31.getConfigurator().apply(pidConfigs);
    m_motor2.getConfigurator().apply(pidConfigs);
  }

  @Override
  public void setNeutralMode(boolean brake) {
    MotorOutputConfigs output = new MotorOutputConfigs();
    output.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    m_motor41.getConfigurator().apply(output);
    m_motor1.getConfigurator().apply(output);
    m_motor9.getConfigurator().apply(output);
    m_motor31.getConfigurator().apply(output);
    m_motor2.getConfigurator().apply(output);
  }
}
