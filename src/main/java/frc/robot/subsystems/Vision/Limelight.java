// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  private static String tableKey = "Vision_";

  public Limelight() {}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Vision_pipeline", LimelightHelpers.getCurrentPipelineIndex("front"));
  }
}

/*
get info on which tags are visible
change pipline based on tag seen
pid to turn to tag
*/
