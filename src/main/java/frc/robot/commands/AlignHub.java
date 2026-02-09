// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

public class AlignHub extends Command {
  /** Creates a new AlignHub. */
  private double tagID;

  private static String name = "front";
  private static String key = "FrontVision_";
  private static Translation2d translation = new Translation2d(0, 0);

  public AlignHub() {
    // Use addRequirements() here to declare subsystem dependencies.
    tagID = -1;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // s_Swerve.resetTurnController();
    // s_Swerve.setTurnControllerGoal(LimelightHelpers.getTX(name));
    LimelightHelpers.setPipelineIndex(name, 0);
    if (tagID == -1
        || tagID == 1
        || tagID == 3
        || tagID == 4
        || tagID == 6
        || tagID == 7
        || tagID == 12
        || tagID == 13
        || tagID == 14
        || tagID == 15
        || tagID == 16
        || tagID == 17
        || tagID == 19
        || tagID == 20
        || tagID == 22
        || tagID == 23
        || tagID == 28
        || tagID == 29
        || tagID == 30
        || tagID == 31
        || tagID == 32) {

      try {
        tagID = LimelightHelpers.getFiducialID(name);
      } catch (Exception e) {
        System.out.println("Error getting fiducial ID");
        tagID = -1;
      }
    }
    if (tagID != -1) {
      SmartDashboard.putNumber(key + "tagID", tagID);
      if (tagID == 2 || tagID == 18) {
        LimelightHelpers.setPipelineIndex(name, 1);
        SmartDashboard.putNumber(key + "pipeline", 1);
      } else if (tagID == 11 || tagID == 27) {
        LimelightHelpers.setPipelineIndex(name, 2);
        SmartDashboard.putNumber(key + "pipeline", 2);
      } else if (tagID == 10 || tagID == 26) {
        LimelightHelpers.setPipelineIndex(name, 3);
        SmartDashboard.putNumber(key + "pipeline", 3);
      } else if (tagID == 9 || tagID == 25) {
        LimelightHelpers.setPipelineIndex(name, 4);
        SmartDashboard.putNumber(key + "pipeline", 4);
      } else if (tagID == 8 || tagID == 24) {
        LimelightHelpers.setPipelineIndex(name, 5);
        SmartDashboard.putNumber(key + "pipeline", 5);
      } else if (tagID == 5 || tagID == 21) {
        LimelightHelpers.setPipelineIndex(name, 6);
        SmartDashboard.putNumber(key + "pipeline", 6);
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double theta = s_Swerve.getTurnPidSpeed; //get theta out from Swerve
    // s_Swerve.drive(translation, theta, false, false);
    /*
    if (s_Swerve.getPidAtGoalYaw()) {
      s_Swerve.stopDriving();
      end(true);
    }
    */
      System.out.println(tagID);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LimelightHelpers.setPipelineIndex(name, 0);
    // s_Swerve.stopDriving();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
