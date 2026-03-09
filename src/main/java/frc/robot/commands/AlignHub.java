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
  @SuppressWarnings("unused")
  private double tagID;

  private static String name = "limelight-front";

  @SuppressWarnings("unused")
  private static Translation2d translation = new Translation2d(0, 0);

  public AlignHub() {
    // Use addRequirements() here to declare subsystem dependencies.
    tagID = LimelightHelpers.getFiducialID(name);
    LimelightHelpers.setPipelineIndex(name, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // s_Swerve.resetTurnController();
    // s_Swerve.setTurnControllerGoal(LimelightHelpers.getTX(name));
    Limelight.setPipeline();
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
    System.out.println("Aligning to hub????????????? we shall see");
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
    // Consider the robot aligned when the Limelight's horizontal offset (tx)
    // is within a small tolerance of zero.
    double tx = LimelightHelpers.getTX(name);
    double alignmentToleranceDegrees = 1.0;
    return Math.abs(tx) <= alignmentToleranceDegrees;
  }
}
