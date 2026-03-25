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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableNumber;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.*;

public class ShooterSubsystem extends SubsystemBase {

  // Motor CAN IDs
  private static final int MOTOR_41_ID = 41;
  private static final int MOTOR_34_ID = 34;
  private static final int MOTOR_9_ID = 9;
  private static final int MOTOR_31_ID = 31;
  private static final int MOTOR_35_ID = 35;

  // Motors
  /** Shooter motor (CAN ID 41) */
  private final TalonFX m_motor41;

  /** Shooter motor (CAN ID 34) */
  private final TalonFX m_motor34;

  /** Shooter motor (CAN ID 9) */
  private final TalonFX m_motor9;

  /** Shooter motor (CAN ID 31) */
  private final TalonFX m_motor31;

  /** Shooter motor (CAN ID 35) */
  private final TalonFX m_motor35;

  // Velocity control request
  private final VelocityVoltage m_velocityRequest;

  // Tunable PID and feedforward values
  private final TunableNumber m_kp = new TunableNumber("Shooter/kP", 0.12);
  private final TunableNumber m_ki = new TunableNumber("Shooter/kI", 0.0);
  private final TunableNumber m_kd = new TunableNumber("Shooter/kD", 0.003);
  private final TunableNumber m_kv = new TunableNumber("Shooter/kV", 0.12);
  private final TunableNumber m_ks = new TunableNumber("Shooter/kS", 0.0);
  private final TunableNumber m_ka = new TunableNumber("Shooter/kA", 0.01);

  // Separate tunable RPM for top and bottom groups
  private final TunableNumber m_topTargetRPM =
      new TunableNumber("Shooter/TopTargetRPM", ShooterConstants.topTESTrpm);
  private final TunableNumber m_bottomTargetRPM =
      new TunableNumber("Shooter/BottomTargetRPM", ShooterConstants.bottomTESTrpm);

  // Tolerance for determining if shooter is at speed (in RPS)
  private static final double VELOCITY_TOLERANCE_RPS = 0.5;

  // Current limit in amps
  private static final double CURRENT_LIMIT = 40.0;

  public ShooterSubsystem() {
    m_motor41 = new TalonFX(MOTOR_41_ID);
    m_motor34 = new TalonFX(MOTOR_34_ID);
    m_motor9 = new TalonFX(MOTOR_9_ID);
    m_motor31 = new TalonFX(MOTOR_31_ID);
    m_motor35 = new TalonFX(MOTOR_35_ID);
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

    // PID + feedforward (Slot 0)
    Slot0Configs pidConfigs = new Slot0Configs();
    pidConfigs.kP = m_kp.get();
    pidConfigs.kI = m_ki.get();
    pidConfigs.kD = m_kd.get();
    pidConfigs.kV = m_kv.get();
    pidConfigs.kS = m_ks.get();
    pidConfigs.kA = m_ka.get();
    config.Slot0 = pidConfigs;

    // Motor output config
    MotorOutputConfigs outputConfig = new MotorOutputConfigs();
    outputConfig.NeutralMode = NeutralModeValue.Coast;

    // Motor 41 -> CounterClockwise Positive
    outputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput = outputConfig;
    m_motor41.getConfigurator().apply(config);

    // Motor 1 -> Clockwise Positive (flipped)
    outputConfig.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput = outputConfig;
    m_motor34.getConfigurator().apply(config);

    // Motors 9, 31, 2 -> CounterClockwise Positive
    outputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput = outputConfig;
    m_motor9.getConfigurator().apply(config);
    m_motor31.getConfigurator().apply(config);
    m_motor35.getConfigurator().apply(config);
  }

  /**
   * Runs top and bottom groups at their respective tunable RPM targets. Top motors (41, 1) run at
   * TopTargetRPM. Bottom motors (9, 31, 2) run at BottomTargetRPM.
   *
   * @param topRPM Desired top-group velocity in RPM.
   */
  public void runAtTargetVelocity(double topRPM) {
    double topRPS = topRPM / 60.0;
    double bottomRPS = m_bottomTargetRPM.get() / 60.0;
    setTopGroupVelocityRPS(topRPS);
    if (isAtSpeedFly(topRPS)) {
      setBottomGroupVelocityRPS(bottomRPS);
    }
  }

  public void runAtTargetVelocity() {
    double topRPS = m_topTargetRPM.get() / 60.0;
    double bottomRPS = m_bottomTargetRPM.get() / 60.0;
    setTopGroupVelocityRPS(topRPS);
    if (isAtSpeedFly(topRPS)) {
      setBottomGroupVelocityRPS(bottomRPS);
    }
  }

  /** Sets top group motors (41, 1) to the given velocity in RPS. */
  public void setBottomGroupVelocityRPS(double rps) {
    m_motor41.setControl(m_velocityRequest.withVelocity(rps));
    m_motor34.setControl(m_velocityRequest.withVelocity(rps));
  }

  /** Sets bottom group motors (9, 31, 2) to the given velocity in RPS. */
  public void setTopGroupVelocityRPS(double rps) {
    m_motor9.setControl(m_velocityRequest.withVelocity(rps));
    m_motor31.setControl(m_velocityRequest.withVelocity(rps));
    m_motor35.setControl(m_velocityRequest.withVelocity(rps));
  }

  /** Sets all shooter motors to the given velocity in RPS. */
  public void setVelocityRPS(double rps) {
    setTopGroupVelocityRPS(rps);
    setBottomGroupVelocityRPS(rps);
  }

  /** Stops all shooter motors. */
  public void stop() {
    m_motor41.stopMotor();
    m_motor34.stopMotor();
    m_motor9.stopMotor();
    m_motor31.stopMotor();
    m_motor35.stopMotor();
  }

  /** Runs all shooters at a percentage of max output (for testing). */
  public void setPercentOutput(double percent) {
    m_motor41.set(percent);
    m_motor34.set(percent);
    m_motor9.set(percent);
    m_motor31.set(percent);
    m_motor35.set(percent);
  }

  /** Runs motors 9, 2, and 31 at percent output. */
  public void setBottomGroupPercent(double percent) {
    m_motor9.set(percent);
    m_motor35.set(percent);
    m_motor31.set(percent);
  }

  /** Runs motors 41 and 1 at percent output. */
  public void setTopGroupPercent(double percent) {
    m_motor41.set(percent);
    m_motor34.set(percent);
  }

  /** Stops motors 9, 2, and 31. */
  public void stopBottomGroup() {
    m_motor9.stopMotor();
    m_motor35.stopMotor();
    m_motor31.stopMotor();
  }

  /** Stops motors 41 and 1. */
  public void stopTopGroup() {
    m_motor41.stopMotor();
    m_motor34.stopMotor();
  }

  // ========== RPM Getters ==========

  public double getMotor41RPM() {
    return m_motor41.getVelocity().getValueAsDouble() * 60.0;
  }

  public double getMotor1RPM() {
    return m_motor34.getVelocity().getValueAsDouble() * 60.0;
  }

  public double getMotor9RPM() {
    return m_motor9.getVelocity().getValueAsDouble() * 60.0;
  }

  public double getMotor31RPM() {
    return m_motor31.getVelocity().getValueAsDouble() * 60.0;
  }

  public double getMotor2RPM() {
    return m_motor35.getVelocity().getValueAsDouble() * 60.0;
  }

  /** Checks if all flywheel motors are within tolerance of their target velocity. */
  public boolean isAtSpeedFly(double top) {
    double topRPS = top;
    if (topRPS == 0.0) {
      return false;
    }

    return (Math.abs(m_motor9.getVelocity().getValueAsDouble() - topRPS) < VELOCITY_TOLERANCE_RPS
        && Math.abs(m_motor31.getVelocity().getValueAsDouble() - topRPS) < VELOCITY_TOLERANCE_RPS
        && Math.abs(m_motor35.getVelocity().getValueAsDouble() - topRPS) < VELOCITY_TOLERANCE_RPS);
  }

  public double getTopTargetRPM() {
    return m_topTargetRPM.get();
  }

  public double getBottomTargetRPM() {
    return m_bottomTargetRPM.get();
  }

  /** Updates PID values from tunable numbers (only Slot0, avoids full reconfigure). */
  private void updatePIDValues() {
    Slot0Configs pidConfigs = new Slot0Configs();
    pidConfigs.kP = m_kp.get();
    pidConfigs.kI = m_ki.get();
    pidConfigs.kD = m_kd.get();
    pidConfigs.kV = m_kv.get();
    pidConfigs.kS = m_ks.get();
    pidConfigs.kA = m_ka.get();

    m_motor41.getConfigurator().apply(pidConfigs);
    m_motor34.getConfigurator().apply(pidConfigs);
    m_motor9.getConfigurator().apply(pidConfigs);
    m_motor31.getConfigurator().apply(pidConfigs);
    m_motor35.getConfigurator().apply(pidConfigs);
  }

  /** Sets the neutral mode for all motors. */
  public void setNeutralMode(NeutralModeValue mode) {
    MotorOutputConfigs output = new MotorOutputConfigs();
    output.NeutralMode = mode;

    m_motor41.getConfigurator().apply(output);
    m_motor34.getConfigurator().apply(output);
    m_motor9.getConfigurator().apply(output);
    m_motor31.getConfigurator().apply(output);
    m_motor35.getConfigurator().apply(output);
  }

  public double getDist() {
    try {
      int a =
          ((int) (LimelightHelpers.getTargetPose3d_CameraSpace("limelight-front").getZ() * 100));
      double b = (double) a;
      b = b / 20.0;
      b = Math.round(b) * 2.0;
      return b / 10.0;
    } catch (Exception e) {
      System.out.println("distance doens't work");
    }
    return -1.0;
  }

  @Override
  public void periodic() {
    // Update PID if tunable numbers changed
    if (m_kp.hasChanged()
        || m_ki.hasChanged()
        || m_kd.hasChanged()
        || m_kv.hasChanged()
        || m_ks.hasChanged()
        || m_ka.hasChanged()) {
      updatePIDValues();
    }
    SmartDashboard.putNumber("Shooter/ForwardDistance", getDist());

    // RPM Telemetry
    SmartDashboard.putNumber("Shooter/Motor41RPM", getMotor41RPM());
    SmartDashboard.putNumber("Shooter/Motor1RPM", getMotor1RPM());
    SmartDashboard.putNumber("Shooter/Motor9RPM", getMotor9RPM());
    SmartDashboard.putNumber("Shooter/Motor31RPM", getMotor31RPM());
    SmartDashboard.putNumber("Shooter/Motor2RPM", getMotor2RPM());
    SmartDashboard.putNumber("Shooter/TopTargetRPM", m_topTargetRPM.get());
    SmartDashboard.putNumber("Shooter/BottomTargetRPM", m_bottomTargetRPM.get());

    // Voltage Telemetry
    SmartDashboard.putNumber(
        "Shooter/Motor41Voltage", m_motor41.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber(
        "Shooter/Motor1Voltage", m_motor34.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber(
        "Shooter/Motor9Voltage", m_motor9.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber(
        "Shooter/Motor31Voltage", m_motor31.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber(
        "Shooter/Motor2Voltage", m_motor35.getMotorVoltage().getValueAsDouble());

    // Current Telemetry
    SmartDashboard.putNumber(
        "Shooter/Motor41Current", m_motor41.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        "Shooter/Motor1Current", m_motor34.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        "Shooter/Motor9Current", m_motor9.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        "Shooter/Motor31Current", m_motor31.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        "Shooter/Motor2Current", m_motor35.getSupplyCurrent().getValueAsDouble());
  }
}
