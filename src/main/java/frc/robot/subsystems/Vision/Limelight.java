// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  private static String tableName = "limelight-front";

  public Limelight() {
    LimelightHelpers.setPipelineIndex(tableName, 0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Vision_tagID", LimelightHelpers.getFiducialID(tableName));
    SmartDashboard.putNumber(
        "Vision_pipeline", LimelightHelpers.getCurrentPipelineIndex(tableName));
  }

  public static void setPipeline() {
    double tag = LimelightHelpers.getFiducialID(tableName);
    if (tag == 2 || tag == 18) {
      LimelightHelpers.setPipelineIndex(tableName, 1);
    } else if (tag == 11 || tag == 27) {
      LimelightHelpers.setPipelineIndex(tableName, 2);
    } else if (tag == 10 || tag == 26) {
      LimelightHelpers.setPipelineIndex(tableName, 3);
    } else if (tag == 9 || tag == 25) {
      LimelightHelpers.setPipelineIndex(tableName, 4);
    } else if (tag == 8 || tag == 24) {
      LimelightHelpers.setPipelineIndex(tableName, 5);
    } else if (tag == 5 || tag == 21) {
      LimelightHelpers.setPipelineIndex(tableName, 6);
    } else {
      LimelightHelpers.setPipelineIndex(tableName, 0);
    }
  }
}
