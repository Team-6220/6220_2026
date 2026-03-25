// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.ArmSubsystem;

/**
 * Moves the arm to a given encoder position and holds it there.
 *
 * <p>!! SET YOUR SETPOINTS CAREFULLY !! The encoder is relative — zero is wherever the arm was when
 * the robot booted. Start with small values and increase gradually while watching the arm.
 */
public class ArmToPositionCommand extends Command {

  private final ArmSubsystem m_arm;
  private final double m_setpoint;

  /**
   * @param arm the arm subsystem
   * @param setpoint target encoder position
   */
  public ArmToPositionCommand(ArmSubsystem arm, double setpoint) {
    m_arm = arm;
    m_setpoint = setpoint;
    addRequirements(m_arm);
  }

  @Override
  public void initialize() {
    m_arm.setGoal(m_setpoint);
    System.out.println("[ArmToPositionCommand] Moving to: " + m_setpoint);
  }

  @Override
  public void execute() {
    m_arm.swingToGoal();
  }

  @Override
  public boolean isFinished() {
    return m_arm.controllerAtGoal();
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.maintain();
    if (interrupted) {
      System.out.println("[ArmToPositionCommand] Interrupted at: " + m_arm.getPosition());
    } else {
      System.out.println("[ArmToPositionCommand] Reached: " + m_setpoint);
    }
  }
}
