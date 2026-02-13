// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Vision.Limelight;

public class AlignHub extends Command {
  /** Creates a new AlignHub. */
  private double tagID;

  private static String name = "front";
  private static String key = "FrontVision_";
  private static Translation2d translation = new Translation2d(0, 0);
  private Limelight s_Limelight;

  public AlignHub(Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Limelight = limelight;
    tagID = s_Limelight.getTagID();
    System.out.print(tagID);
    LimelightHelpers.setPipelineIndex(name, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // s_Swerve.resetTurnController();
    // s_Swerve.setTurnControllerGoal(LimelightHelpers.getTX(name));

    if (tagID == 2.0 || tagID == 18.0) {
      LimelightHelpers.setPipelineIndex(name, 1);
    } else if (tagID == 11.0 || tagID == 27.0) {
      LimelightHelpers.setPipelineIndex(name, 2);
    } else if (tagID == 10.0 || tagID == 26.0) {
      LimelightHelpers.setPipelineIndex(name, 3);
    } else if (tagID == 9.0 || tagID == 25.0) {
      LimelightHelpers.setPipelineIndex(name, 4);
    } else if (tagID == 8.0 || tagID == 24.0) {
      LimelightHelpers.setPipelineIndex(name, 5);
    } else if (tagID == 5.0 || tagID == 21.0) {
      LimelightHelpers.setPipelineIndex(name, 6);
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
    System.out.println(tagID + "  pipline: " + LimelightHelpers.getCurrentPipelineIndex(name));
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
