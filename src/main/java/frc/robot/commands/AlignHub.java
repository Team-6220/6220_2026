// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.VisionConstants;
import frc.robot.subsystems.Drive.Swerve;
import frc.robot.subsystems.Vision.Limelight;

public class AlignHub extends Command {
  /** Creates a new AlignHub. */

  private static String name = "limelight-front";
  private static Translation2d translation = new Translation2d(0, 0);
  private Swerve s_Swerve;

  public AlignHub(Swerve s_Swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = s_Swerve;
    LimelightHelpers.setPipelineIndex(name, 0);
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Swerve.resetTurnController();
    Limelight.setPipeline();
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Swerve.drive(translation, LimelightHelpers.getTX(name), false, false);
    // double theta = s_Swerve.getTurnPidSpeed(); //get theta out from Swerve
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
    double tx = LimelightHelpers.getTX(name);
    double alignmentToleranceDegrees = 1.0;
    return Math.abs(tx) <= alignmentToleranceDegrees;
  }
}
