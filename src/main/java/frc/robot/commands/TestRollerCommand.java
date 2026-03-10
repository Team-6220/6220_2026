// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.RollerSubsystem;

/**
 * Spins the roller motor in intake or outtake direction for testing.
 *
 * <p>Suggested bindings in RobotContainer: // Intake while held new JoystickButton(testController,
 * XboxController.Button.kX.value) .whileTrue(new TestRollerCommand(roller, true));
 *
 * <p>// Outtake while held new JoystickButton(testController, XboxController.Button.kY.value)
 * .whileTrue(new TestRollerCommand(roller, false));
 */
public class TestRollerCommand extends Command {

  private final RollerSubsystem roller;
  private final boolean intake; // true = intake, false = outtake

  public TestRollerCommand(boolean intake) {
    this.roller = RollerSubsystem.getInstance();
    this.intake = intake;
    addRequirements(roller);
  }

  @Override
  public void initialize() {
    System.out.println("[TestRollerCommand] " + (intake ? "Intaking" : "Outtaking"));
  }

  @Override
  public void execute() {
    roller.simpleDrive(intake ? 1 : -1);
  }

  @Override
  public void end(boolean interrupted) {
    roller.stop();
    System.out.println("[TestRollerCommand] Stopping roller");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
