// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private final SparkMax rollerMotor;
  private final SparkMax armMotor;


  private static final class IntakeConstants{
    public static final int ROLLERMOTORID = 0;
    public static final int ARMMOTORID = 0;

    public static final double INTAKESPEED = 0.75;
    public static final double ARMSPEED = 0.5;
    public static final int CURRENTLIMIT = 30;
  }

  public IntakeSubsystem() {
    rollerMotor = new SparkMax(IntakeConstants.ROLLERMOTORID, MotorType.kBrushless);
    rollerMotor.setInverted(false);
    rollerMotor.setIdleMode(SparkMax.IdleMode.kCoast);

    armMotor = new SparkMax(IntakeConstants.ARMMOTORID, MotorType.kBrushless);
    armMotor.setInverted(false);
    armMotor.setIdleMode(SparkMax.IdleMode.kCoast);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
