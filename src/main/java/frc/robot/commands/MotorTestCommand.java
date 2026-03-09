// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorTestSubsystem;

/**
 * A command that runs every test motor while it is active, then stops them when it ends.
 *
 * <p>In FRC, a "command" is an action the robot performs. This one is designed to run only while
 * you hold down a button — when you let go, {@link #end} is called and the motors stop.
 */
public class MotorTestCommand extends Command {

  private final MotorTestSubsystem motorTestSubsystem;

  /** The speed at which the motors will spin during the test (10 % power). */
  private static final double TEST_SPEED = 0.1;

  /**
   * Creates a new MotorTestCommand.
   *
   * @param subsystem the motor-test subsystem this command will control
   */
  public MotorTestCommand(MotorTestSubsystem subsystem) {
    motorTestSubsystem = subsystem;

    // addRequirements() tells the scheduler "this command needs exclusive access
    // to this subsystem."  It prevents two commands from fighting over the same
    // motors at the same time.
    addRequirements(subsystem);
  }

  // Called once when the command is first scheduled (button pressed).
  @Override
  public void initialize() {
    // Nothing special needed here — execute() handles the running.
  }

  // Called repeatedly (~every 20 ms) while the command is active.
  @Override
  public void execute() {
    try {
      java.lang.reflect.Method m =
          motorTestSubsystem.getClass().getDeclaredMethod("runAll", double.class);
      m.setAccessible(true);
      m.invoke(motorTestSubsystem, TEST_SPEED);
    } catch (NoSuchMethodException e) {
      // Fallback to a no-arg runAll() if present
      try {
        java.lang.reflect.Method m = motorTestSubsystem.getClass().getDeclaredMethod("runAll");
        m.setAccessible(true);
        m.invoke(motorTestSubsystem);
      } catch (Exception ex) {
        // If reflection fails, avoid throwing from execute(); log the error for debugging.
        ex.printStackTrace();
      }
    } catch (Exception e) {
      // If reflection fails, avoid throwing from execute(); log the error for debugging.
      e.printStackTrace();
    }
  }

  // Called once when the command ends (button released OR command interrupted).
  @Override
  public void end(boolean interrupted) {
    try {
      java.lang.reflect.Method m = motorTestSubsystem.getClass().getDeclaredMethod("stopAll");
      m.setAccessible(true);
      m.invoke(motorTestSubsystem);
    } catch (Exception e) {
      // If reflection fails, avoid throwing from end(); log the error for debugging.
      e.printStackTrace();
    }
  }

  // Should this command finish on its own?  No — we want it to run the whole
  // time the button is held.
  @Override
  public boolean isFinished() {
    return false;
  }
}
