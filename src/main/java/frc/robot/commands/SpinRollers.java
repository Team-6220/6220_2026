// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinRollers extends Command {
  /** Creates a new SpinRollers. */
  private IntakeSubsystem intake;

  private XboxController controller;

  public SpinRollers(XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = IntakeSubsystem.getInstance();
    this.controller = controller;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double input = controller.getLeftY();
    if (Math.abs(input) < 0.1) {
      input = 0;
    }
    intake.simpleDriveRoller(input);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.simpleDriveRoller(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
