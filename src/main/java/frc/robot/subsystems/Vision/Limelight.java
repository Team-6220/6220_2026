// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  private double tagID;

  private static String tableKey = "Vision_";

  public Limelight() {
    tagID = -1.0;
    LimelightHelpers.setPipelineIndex("front", 0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Vision_tagID", getTagID());
    SmartDashboard.putNumber("Vision_pipeline", LimelightHelpers.getCurrentPipelineIndex("front"));
  }

  public static double getTagID() {
    LimelightResults a = LimelightHelpers.getLatestResults("front");
    try {
      return LimelightHelpers.getFiducialID("front");
    } catch (Exception e) {
      System.out.println("No tag detected");
    }
    return -1.0;
  } 

  public static void setPipeline() {
    double tag = getTagID();
    if (tag == 2 || tag == 18) {
      LimelightHelpers.setPipelineIndex("front", 1);
    } 
    else if (tag == 11 || tag == 27) {
      LimelightHelpers.setPipelineIndex("front", 2);
    } 
    else if (tag == 10 || tag == 26) {
      LimelightHelpers.setPipelineIndex("front", 3);
    } 
    else if (tag == 9 || tag == 25) {
      LimelightHelpers.setPipelineIndex("front", 4);
    } 
    else if (tag == 8 || tag == 24) {
      LimelightHelpers.setPipelineIndex("front", 5);
    } 
    else if (tag == 5 || tag == 21) {
      LimelightHelpers.setPipelineIndex("front", 6);
    }
    else {
      LimelightHelpers.setPipelineIndex("front", 0);
    }
  }
}

/*
get info on which tags are visible
change pipline based on tag seen
pid to turn to tag
*/
