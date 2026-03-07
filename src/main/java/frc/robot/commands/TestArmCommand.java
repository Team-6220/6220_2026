// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.ArmSubsystem;
import java.util.function.DoubleSupplier;

/**
 * Manually drives the arm motor using a joystick axis. Bind to a button/trigger so the arm stops
 * (0V) when released.
 *
 * <p>Suggested binding in RobotContainer: new JoystickButton(testController,
 * XboxController.Button.kLeftBumper.value) .whileTrue(new TestArmCommand(arm, () ->
 * testController.getLeftY()));
 */
public class TestArmCommand extends Command {

  private final ArmSubsystem arm;
  private final DoubleSupplier speedSupplier;

  public TestArmCommand(ArmSubsystem arm, DoubleSupplier speedSupplier) {
    this.arm = arm;
    this.speedSupplier = speedSupplier;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    System.out.println("[TestArmCommand] Starting arm manual drive");
  }

  @Override
  public void execute() {
    // Apply a small deadband so the arm doesn't drift on stick release
    double raw = speedSupplier.getAsDouble();
    double output = Math.abs(raw) > 0.08 ? raw : 0.0;
    arm.simpleDrive(output);
  }

  @Override
  public void end(boolean interrupted) {
    arm.simpleDrive(0);
    System.out.println("[TestArmCommand] Stopping arm");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
