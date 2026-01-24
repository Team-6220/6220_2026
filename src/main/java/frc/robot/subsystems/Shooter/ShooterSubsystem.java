// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableNumber;
import frc.robot.subsystems.Shooter.ShooterIO.ShooterIOInputs;

public class ShooterSubsystem extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputs inputs = new ShooterIOInputs();


  private final RelativeEncoder m_topEncoder;
  private final RelativeEncoder m_bottomEncoder;

  private final PIDController m_topPidController;
  private final PIDController m_bottomPidController;

  private final TunableNumber m_kp = new TunableNumber("Shooter/kp", 0.0);
  private final TunableNumber m_ki = new TunableNumber("Shooter/ki", 0.0);
  private final TunableNumber m_kd = new TunableNumber("Shooter/kd", 0.0);
  private final TunableNumber m_kv = new TunableNumber("Shooter/kv", 0.0);


  private double m_targetRPM = 0.0;


  // public ShooterSubsystem(ShooterIO io) {
  //   this.io = io;

    
  // }
  public ShooterSubsystem() {
    final SparkMax m_topMotorShooter = new SparkMax(ShooterIO.Motor1ID, MotorType.kBrushless);
    final SparkMax m_bottomMotorShooter = new SparkMax(ShooterIO.Motor2ID, MotorType.kBrushless);
    
    m_bottomMotorShooter.setInverted(true);
    m_bottomMotorShooter.setInverted(false);
    

  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // TODO: add staged logic 
  }

  // TODO: add setTargetRPM, isReady, etc.
}
