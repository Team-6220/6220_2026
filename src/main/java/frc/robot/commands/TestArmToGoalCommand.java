// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.ArmSubsystem;

/**
 * Moves the arm to a target position using the ProfiledPID + ArmFeedforward controllers. Ends
 * automatically when the controller reports atGoal(). Useful for verifying your PID/FF constants
 * are tuned correctly.
 *
 * <p>Suggested binding in RobotContainer: new JoystickButton(testController,
 * XboxController.Button.kRightBumper.value) .onTrue(new TestArmToGoalCommand(arm, 45.0)); // move
 * to 45 degrees
 */
public class TestArmToGoalCommand extends Command {

  private final ArmSubsystem arm;
  private final double goalDegrees;

  public TestArmToGoalCommand(ArmSubsystem arm, double goalDegrees) {
    this.arm = arm;
    this.goalDegrees = goalDegrees;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    System.out.println("[TestArmToGoalCommand] Moving arm to goal: " + goalDegrees + " deg");
    arm.setGoal(goalDegrees);
  }

  @Override
  public void execute() {
    arm.swingToGoal();
  }

  @Override
  public void end(boolean interrupted) {
    arm.simpleDrive(0);
    if (interrupted) {
      System.out.println("[TestArmToGoalCommand] Interrupted before reaching goal");
    } else {
      System.out.println("[TestArmToGoalCommand] Reached goal: " + goalDegrees + " deg");
    }
  }

  @Override
  public boolean isFinished() {
    return arm.controllerAtGoal();
  }
}
