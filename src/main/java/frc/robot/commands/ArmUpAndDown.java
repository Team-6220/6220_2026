// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake.ArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmUpAndDown extends Command {
  /** Creates a new ArmUpAndDown. */
  private ArmSubsystem arm;
  private final double up = -10.0;
  private final double down = -29.785476684570312;
  private boolean upPosition;

  public ArmUpAndDown(ArmSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setGoal(up);
    upPosition = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.swingToGoal();
    if (Math.abs(arm.getPosition() - up) < 1 && upPosition) {
      arm.setGoal(down);
      upPosition = false;
    } else if (Math.abs(down - arm.getPosition()) < 1 && !upPosition) {
      arm.setGoal(up);
      upPosition = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.maintain();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
