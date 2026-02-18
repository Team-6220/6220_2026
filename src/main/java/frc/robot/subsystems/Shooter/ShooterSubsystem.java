// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableNumber;
import frc.robot.subsystems.Shooter.ShooterIO.ShooterIOInputs;

public class ShooterSubsystem extends SubsystemBase {
  private  ShooterIO io;
  private  ShooterIOInputs inputs = new ShooterIOInputs();


  private  RelativeEncoder m_topEncoder;
  private  RelativeEncoder m_bottomEncoder;

  private  PIDController m_topPidController;
  private  PIDController m_bottomPidController;

  private  final TunableNumber m_kp = new TunableNumber("Shooter/kp", 0.0);
  private  final TunableNumber m_ki = new TunableNumber("Shooter/ki", 0.0);
  private  final TunableNumber m_kd = new TunableNumber("Shooter/kd", 0.0);
  private  final TunableNumber m_kv = new TunableNumber("Shooter/kv", 0.0);
  private  final TunableNumber m_RPM = new TunableNumber("Shooter/RPM", 0.0);

  private double m_targetRPM = 0.0;

    final TalonFX m_topMotorShooter = new TalonFX(ShooterIO.Motor1ID);
    final TalonFX m_bottomMotorShooter = new TalonFX(ShooterIO.Motor2ID);
  // public ShooterSubsystem(ShooterIO io) {
  //   this.io = io;

    
  // }
  public ShooterSubsystem() {

    
    
    CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
    limits.SupplyCurrentLimitEnable = true;
    limits.SupplyCurrentLimit = 40;

m_topMotorShooter.getConfigurator().apply(limits);
m_bottomMotorShooter.getConfigurator().apply(limits);
  MotorOutputConfigs output = new MotorOutputConfigs();
  output.Inverted = InvertedValue.Clockwise_Positive;

  m_topMotorShooter.getConfigurator().apply(output);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // TODO: add staged logic 
  }

  // TODO: add setTargetRPM, isReady, etc.
}
