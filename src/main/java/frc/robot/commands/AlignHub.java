// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
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
    Limelight.setPipeline();
    s_Swerve.resetTurnController();
    s_Swerve.setTurnControllerGoal(
        Degree.of(LimelightHelpers.getTX(name) + s_Swerve.getHeadingDegrees()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Swerve.setTurnControllerGoal(
        Degree.of(LimelightHelpers.getTX(name) + s_Swerve.getHeadingDegrees()));
    s_Swerve.drive(translation, s_Swerve.getTurnPidSpeed(), false, false);
    System.out.println("Aligning to hub????????????? we shall see");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LimelightHelpers.setPipelineIndex(name, 0);
    System.out.println("DONE ALIGNINGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Consider the robot aligned when the Limelight's horizontal offset (tx)
    // is within a small tolerance of zero.
    return false;
  }
}
