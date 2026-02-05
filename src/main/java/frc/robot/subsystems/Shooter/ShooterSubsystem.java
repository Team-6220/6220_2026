// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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


  // public ShooterSubsystem(ShooterIO io) {
  //   this.io = io;
  
    
  // }
  public ShooterSubsystem() {
   
    
   // m_bottomMotorShooter.setInverted(true);
   // m_bottomMotorShooter.setInverted(false);
    

  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    
    //ℹ️❤️📋
    if (m_kp.hasChanged() || m_ki.hasChanged() || m_kd.hasChanged()) {
      m_Controller.setPID(m_kp.get(), m_ki.get(), m_kd.get());
      System.out.println(
          "*********PID With new P:"
              + m_kp.get()
              + "ki:"
              + m_ki.get()
              + "kd:"
              + m_kd.get());
    }
  }
  public boolean spinUp(){
    double averagedSpeed=(m_MotorShooter1.getVelocity().getValue().abs(RPM) + m_MotorShooter2.getVelocity().getValue().abs(RPM) + m_MotorShooter3.getVelocity().getValue().abs(RPM))/3;
    if(averagedSpeed<m_targetRPM){
      m_MotorShooter1.set(1);
      m_MotorShooter2.set(1);
      m_MotorShooter3.set(1);
    }
    else{return true;}
  }
  public void kick(boolean ctrl){
    m_kicker1.set(ctrl ? 1 : 0);
    m_kicker2.set(ctrl ? 1 : 0);
  }
  public void setHood(double position){
    m_Controller.setGoal(position);
  }
  // TODO: add setTargetRPM, isReady, etc.
}