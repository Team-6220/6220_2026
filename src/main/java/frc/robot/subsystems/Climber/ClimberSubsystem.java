// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableNumber;
import frc.robot.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  public boolean leftClimberAtBottom = false;
  public boolean leftClimberAtTop = false;
  // public boolean rightClimberAtBottom = false; // Uncomment when right climber is connected
  // public boolean rightClimberAtTop = false; // Uncomment when right climber is connected

  private final ClimberIO io;
  private final ClimberIO.ClimberIOInputs inputs = new ClimberIO.ClimberIOInputs();

  // Tunable thresholds for stall detection
  private final TunableNumber stallCurrentBottom =
      new TunableNumber("Climber/StallCurrentBottom", ClimberConstants.STALL_CURRENT_THRESHOLD_BOTTOM);
  private final TunableNumber stallCurrentTop =
      new TunableNumber("Climber/StallCurrentTop", ClimberConstants.STALL_CURRENT_THRESHOLD_TOP);

  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
    // Initialize both servos to retracted position (0.0) on robot startup
    // This ensures the first toggle will always deploy (move to 1.0)
    io.setServoPositions(0.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // Log current telemetry for debugging - Left Climber
    SmartDashboard.putNumber("Climber/Left/MotorCurrent", inputs.leftMotorCurrent);
    SmartDashboard.putNumber("Climber/Left/MotorVoltage", inputs.leftMotorVoltage);
    SmartDashboard.putNumber("Climber/Left/MotorVelocity", inputs.leftMotorVelocity);
    SmartDashboard.putBoolean("Climber/Left/AtBottom", leftClimberAtBottom);
    SmartDashboard.putBoolean("Climber/Left/AtTop", leftClimberAtTop);

    // Debug: Stall detection values
    SmartDashboard.putNumber("Climber/Left/AbsVelocity", Math.abs(inputs.leftMotorVelocity));
    SmartDashboard.putBoolean(
        "Climber/Left/CurrentAboveBottomThreshold",
        inputs.leftMotorCurrent > getStallCurrentThresholdBottom());
    SmartDashboard.putBoolean(
        "Climber/Left/CurrentAboveTopThreshold",
        inputs.leftMotorCurrent > getStallCurrentThresholdTop());
    SmartDashboard.putBoolean(
        "Climber/Left/VelocityAboveThreshold",
        Math.abs(inputs.leftMotorVelocity) > ClimberConstants.VELOCITY_THRESHOLD);
    SmartDashboard.putNumber(
        "Climber/StallCurrentThresholdBottom", getStallCurrentThresholdBottom());
    SmartDashboard.putNumber(
        "Climber/StallCurrentThresholdTop", getStallCurrentThresholdTop());
    SmartDashboard.putNumber("Climber/VelocityThreshold", ClimberConstants.VELOCITY_THRESHOLD);

    // Log current telemetry for debugging - Right Climber (when connected)
    // SmartDashboard.putNumber("Climber/Right/MotorCurrent", inputs.rightMotorCurrent);
    // SmartDashboard.putNumber("Climber/Right/MotorVoltage", inputs.rightMotorVoltage);
    // SmartDashboard.putNumber("Climber/Right/MotorVelocity", inputs.rightMotorVelocity);
    // SmartDashboard.putBoolean("Climber/Right/AtBottom", rightClimberAtBottom);
    // SmartDashboard.putBoolean("Climber/Right/AtTop", rightClimberAtTop);

    // Servo telemetry
    SmartDashboard.putNumber("Climber/Servo/LeftPosition", inputs.leftServoPosition);
    SmartDashboard.putNumber("Climber/Servo/RightPosition", inputs.rightServoPosition);
  }

  /**
   * Get the current bottom stall current threshold (tunable)
   */
  private double getStallCurrentThresholdBottom() {
    return stallCurrentBottom.get();
  }

  /**
   * Get the current top stall current threshold (tunable)
   */
  private double getStallCurrentThresholdTop() {
    return stallCurrentTop.get();
  }

  /**
   * Drive the climber motor at a specified speed (manual control)
   *
   * @param speed Motor speed from -0.75 to 0.75
   * @param controlLeft Whether to control the left climber
   * @param controlRight Whether to control the right climber
   */
  public void setMotorSpeed(double speed, boolean controlLeft, boolean controlRight) {
    // Clamp to safe range
    speed = Math.max(-0.75, Math.min(0.75, speed));

    // Handle left climber
    if (controlLeft) {
      // Block downward movement if left climber already at bottom
      if (leftClimberAtBottom && speed < 0) {
        io.setLeftMotor(0.0);
      }
      // Block upward movement if left climber already at top
      else if (leftClimberAtTop && speed > 0) {
        io.setLeftMotor(0.0);
      } else {
        // If moving upward, clear the at-bottom flag
        if (speed > 0) {
          leftClimberAtBottom = false;
        }
        // If moving downward, clear the at-top flag
        if (speed < 0) {
          leftClimberAtTop = false;
        }
        io.setLeftMotor(speed);
      }
    } else {
      io.setLeftMotor(0.0);
    }

    // Handle right climber (when connected)
    // if (controlRight) {
    //   // Block downward movement if right climber already at bottom
    //   if (rightClimberAtBottom && speed < 0) {
    //     io.setRightMotor(0.0);
    //   }
    //   // Block upward movement if right climber already at top
    //   else if (rightClimberAtTop && speed > 0) {
    //     io.setRightMotor(0.0);
    //   } else {
    //     // If moving upward, clear the at-bottom flag
    //     if (speed > 0) {
    //       rightClimberAtBottom = false;
    //     }
    //     // If moving downward, clear the at-top flag
    //     if (speed < 0) {
    //       rightClimberAtTop = false;
    //     }
    //     io.setRightMotor(speed);
    //   }
    // } else {
    //   io.setRightMotor(0.0);
    // }
  }

  /**
   * Automatically zero the climber by driving down until stall current is detected. The motor will
   * be driven downward until current exceeds the stall threshold, indicating the climber has hit
   * the bottom.
   *
   * @param controlLeft Whether to control the left climber
   * @param controlRight Whether to control the right climber
   */
  public void zeroClimberToBottom(boolean controlLeft, boolean controlRight) {
    // Drive downward (negative speed)
    double downwardSpeed = -0.75;

    // Handle left climber
    if (controlLeft) {
      // If already at bottom, don't allow further downward movement
      if (leftClimberAtBottom) {
        io.setLeftMotor(0.0);
      } else {
        // Clear the at-top flag when moving downward
        leftClimberAtTop = false;
        // Check if left motor is stalled (high current draw)
        if (inputs.leftMotorCurrent > getStallCurrentThresholdBottom()
            && Math.abs(inputs.leftMotorVelocity) > ClimberConstants.VELOCITY_THRESHOLD) {
          // Motor is stalled, we've hit bottom - stop and set flag
          io.setLeftMotor(0.0);
          leftClimberAtBottom = true;
        } else {
          // Continue driving downward
          io.setLeftMotor(downwardSpeed);
        }
      }
    } else {
      io.setLeftMotor(0.0);
    }

    // Handle right climber (when connected)
    // if (controlRight) {
    //   // If already at bottom, don't allow further downward movement
    //   if (rightClimberAtBottom) {
    //     io.setRightMotor(0.0);
    //   } else {
    //     // Clear the at-top flag when moving downward
    //     rightClimberAtTop = false;
    //     // Check if right motor is stalled (high current draw)
    //     if (inputs.rightMotorCurrent > getStallCurrentThresholdBottom()
    //         && Math.abs(inputs.rightMotorVelocity) > ClimberConstants.VELOCITY_THRESHOLD) {
    //       // Motor is stalled, we've hit bottom - stop and set flag
    //       io.setRightMotor(0.0);
    //       rightClimberAtBottom = true;
    //     } else {
    //       // Continue driving downward
    //       io.setRightMotor(downwardSpeed);
    //     }
    //   }
    // } else {
    //   io.setRightMotor(0.0);
    // }
  }

  /**
   * Automatically max the climber by driving up until stall current is detected. The motor will be
   * driven upward until current exceeds the stall threshold, indicating the climber has hit the
   * top.
   *
   * @param controlLeft Whether to control the left climber
   * @param controlRight Whether to control the right climber
   */
  public void maxClimberToTop(boolean controlLeft, boolean controlRight) {
    // Drive upward (positive speed)
    double upwardSpeed = 0.75;

    // Handle left climber
    if (controlLeft) {
      // If already at top, don't allow further upward movement
      if (leftClimberAtTop) {
        io.setLeftMotor(0.0);
      } else {
        // Clear the at-bottom flag when moving upward
        leftClimberAtBottom = false;
        // Check if left motor is stalled (high current draw only, no velocity check)
        if (inputs.leftMotorCurrent > getStallCurrentThresholdTop()) {
          // Motor is stalled, we've hit top - stop and set flag
          io.setLeftMotor(0.0);
          leftClimberAtTop = true;
        } else {
          // Continue driving upward
          io.setLeftMotor(upwardSpeed);
        }
      }
    } else {
      io.setLeftMotor(0.0);
    }

    // Handle right climber (when connected)
    // if (controlRight) {
    //   // If already at top, don't allow further upward movement
    //   if (rightClimberAtTop) {
    //     io.setRightMotor(0.0);
    //   } else {
    //     // Clear the at-bottom flag when moving upward
    //     rightClimberAtBottom = false;
    //     // Check if right motor is stalled (high current draw only, no velocity check)
    //     if (inputs.rightMotorCurrent > getStallCurrentThresholdTop()) {
    //       // Motor is stalled, we've hit top - stop and set flag
    //       io.setRightMotor(0.0);
    //       rightClimberAtTop = true;
    //     } else {
    //       // Continue driving upward
    //       io.setRightMotor(upwardSpeed);
    //     }
    //   }
    // } else {
    //   io.setRightMotor(0.0);
    // }
  }

  /**
   * Check if climber is at zero position (stalled)
   *
   * @param checkLeft Whether to check the left climber
   * @param checkRight Whether to check the right climber
   * @return true if all checked climber(s) are at zero
   */
  public boolean isAtZero(boolean checkLeft, boolean checkRight) {
    boolean leftAtZero = false;

    if (checkLeft) {
      leftAtZero =
          inputs.leftMotorCurrent > getStallCurrentThresholdBottom()
              && inputs.leftMotorVelocity < ClimberConstants.VELOCITY_THRESHOLD;

      // Update the left climber at-bottom flag when we detect zero
      if (leftAtZero) {
        leftClimberAtBottom = true;
      }
    }

    // Check right climber when connected
    // boolean rightAtZero = false;
    // if (checkRight) {
    //   rightAtZero =
    //       inputs.rightMotorCurrent > getStallCurrentThresholdBottom()
    //           && inputs.rightMotorVelocity < ClimberConstants.VELOCITY_THRESHOLD;
    //
    //   // Update the right climber at-bottom flag when we detect zero
    //   if (rightAtZero) {
    //     rightClimberAtBottom = true;
    //   }
    // }

    // Return true if all checked climber(s) are at zero
    if (checkLeft && checkRight) {
      return leftAtZero; // && rightAtZero; // Uncomment when right climber is connected
    } else if (checkLeft) {
      return leftAtZero;
    } else if (checkRight) {
      // return rightAtZero; // Uncomment when right climber is connected
      return false; // Placeholder until right climber is connected
    }
    return false;
  }

  /**
   * Check if climber is at max position (stalled at top)
   *
   * @param checkLeft Whether to check the left climber
   * @param checkRight Whether to check the right climber
   * @return true if all checked climber(s) are at max
   */
  public boolean isAtTop(boolean checkLeft, boolean checkRight) {
    boolean leftAtTop = false;

    if (checkLeft) {
      // Check only current, no velocity requirement for top detection
      leftAtTop = inputs.leftMotorCurrent > getStallCurrentThresholdTop();

      // Update the left climber at-top flag when we detect max
      if (leftAtTop) {
        leftClimberAtTop = true;
      }
    }

    // Check right climber when connected
    // boolean rightAtTop = false;
    // if (checkRight) {
    //   // Check only current, no velocity requirement for top detection
    //   rightAtTop = inputs.rightMotorCurrent > getStallCurrentThresholdTop();
    //
    //   // Update the right climber at-top flag when we detect max
    //   if (rightAtTop) {
    //     rightClimberAtTop = true;
    //   }
    // }

    // Return true if all checked climber(s) are at max
    if (checkLeft && checkRight) {
      return leftAtTop; // && rightAtTop; // Uncomment when right climber is connected
    } else if (checkLeft) {
      return leftAtTop;
    } else if (checkRight) {
      // return rightAtTop; // Uncomment when right climber is connected
      return false; // Placeholder until right climber is connected
    }
    return false;
  }

  /**
   * Set both climber servo positions simultaneously
   *
   * @param position Servo position (0.0 to 1.0)
   */
  public void setServoPositions(double position) {
    io.setServoPositions(position);
  }

  /**
   * Get the current left servo position
   *
   * @return Current position from 0.0 to 1.0
   */
  public double getLeftServoPosition() {
    return inputs.leftServoPosition;
  }

  /**
   * Get the current right servo position
   *
   * @return Current position from 0.0 to 1.0
   */
  public double getRightServoPosition() {
    return inputs.rightServoPosition;
  }

  /** Stop the climber motor */
  public void stop() {
    io.setLeftMotor(0.0);
    // io.setRightMotor(0.0); // Uncomment when right climber is connected
  }

  /** Reset the AtTop and AtBottom limit flags */
  public void resetLimitFlags() {
    leftClimberAtBottom = false;
    leftClimberAtTop = false;
    // rightClimberAtBottom = false; // Uncomment when right climber is connected
    // rightClimberAtTop = false; // Uncomment when right climber is connected
  }
}
