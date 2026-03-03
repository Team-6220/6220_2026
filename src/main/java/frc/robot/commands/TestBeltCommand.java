// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.BeltSubsystem;

/**
 * Runs the belt motor at a fixed speed for testing. Pass a positive speed to run forward, negative
 * to run backward.
 *
 * <p>Suggested bindings in RobotContainer: // Belt forward while held new
 * JoystickButton(testController, XboxController.Button.kA.value) .whileTrue(new
 * TestBeltCommand(belt, 0.5));
 *
 * <p>// Belt backward while held new JoystickButton(testController, XboxController.Button.kB.value)
 * .whileTrue(new TestBeltCommand(belt, -0.5));
 */
public class TestBeltCommand extends Command {

  private final BeltSubsystem belt;
  private final double speed; // -1.0 to 1.0

  public TestBeltCommand(BeltSubsystem belt, double speed) {
    this.belt = belt;
    this.speed = speed;
    addRequirements(belt);
  }

  @Override
  public void initialize() {
    System.out.println("[TestBeltCommand] Starting belt at speed: " + speed);
  }

  @Override
  public void execute() {
    belt.simpleDrive(speed);
  }

  @Override
  public void end(boolean interrupted) {
    belt.stop();
    System.out.println("[TestBeltCommand] Stopping belt");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
