// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A subsystem for testing motors one at a time. Each press of the Start button stops the current
 * motor and starts the next one in the list. One more press after the last motor stops everything.
 *
 * <p>This is a temporary diagnostic tool — remove it before competition.
 */
public class MotorTestSubsystem extends SubsystemBase {

  // ---------------------------------------------------------------------------
  // Falcon 500 motors (built-in TalonFX controller)
  // ---------------------------------------------------------------------------

  /** Shooter — left. */
  private final TalonFX falconShooter9 = new TalonFX(9);

  /** Shooter — right. */
  private final TalonFX falconShooter35 = new TalonFX(35);

  /** Shooter — top. */
  private final TalonFX falconShooter31 = new TalonFX(31);

  /** Rear shooter assembly — feed. */
  private final TalonFX falconRear34 = new TalonFX(34);

  /** Rear shooter assembly — pivot. */
  private final TalonFX falconRear41 = new TalonFX(41);

  // ---------------------------------------------------------------------------
  // NEO 550 motors (SparkMax controller)
  // ---------------------------------------------------------------------------

  /** Intake. */
  private final SparkMax neo550Intake15 = new SparkMax(15, MotorType.kBrushless);

  /** Rear shooter assembly — roller. */
  private final SparkMax neo550Rear19 = new SparkMax(19, MotorType.kBrushless);

  /** Total number of motors in the test cycle. */
  private static final int MOTOR_COUNT = 7;

  /** The speed at which each motor spins during its test (10% power). */
  private static final double TEST_SPEED = 0.1;

  /**
   * Tracks which motor is currently being tested. A value of -1 or anything >= MOTOR_COUNT means
   * "nothing is running."
   */
  private int currentMotorIndex = -1;

  /** Creates a new MotorTestSubsystem. */
  public MotorTestSubsystem() {}

  /**
   * Called once every ~20 ms. Pushes the active motor index to the dashboard so you can see which
   * motor is running without staring at the robot.
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("MotorTest/ActiveIndex", currentMotorIndex);
  }

  /**
   * Stops whatever motor is currently running, advances to the next one, and starts it. If we've
   * already tested the last motor, everything stops and the cycle resets so you can start over.
   *
   * <p>This is the method called by the Start-button binding in RobotContainer.
   */
  public void startNextMotorTest() {
    stopAll();
    currentMotorIndex++;

    if (currentMotorIndex >= MOTOR_COUNT) {
      currentMotorIndex = -1;
      return;
    }

    switch (currentMotorIndex) {
      case 0:
        falconShooter9.set(TEST_SPEED);
        SmartDashboard.putString("MotorTest/Running", "Shooter Falcon (CAN 9)");
        break;
      case 1:
        falconShooter35.set(TEST_SPEED);
        SmartDashboard.putString("MotorTest/Running", "Shooter Falcon (CAN 35)");
        break;
      case 2:
        falconShooter31.set(TEST_SPEED);
        SmartDashboard.putString("MotorTest/Running", "Shooter Falcon (CAN 31)");
        break;
      case 3:
        falconRear34.set(TEST_SPEED);
        SmartDashboard.putString("MotorTest/Running", "Rear Assembly Falcon (CAN 34)");
        break;
      case 4:
        falconRear41.set(TEST_SPEED);
        SmartDashboard.putString("MotorTest/Running", "Rear Assembly Falcon (CAN 41)");
        break;
      case 5:
        neo550Intake15.set(TEST_SPEED);
        SmartDashboard.putString("MotorTest/Running", "Intake NEO 550 (CAN 15)");
        break;
      case 6:
        neo550Rear19.set(TEST_SPEED);
        SmartDashboard.putString("MotorTest/Running", "Rear Assembly NEO 550 (CAN 19)");
        break;
      default:
        stopAll();
        break;
    }
  }

  /** Stops every motor immediately. */
  public void stopAll() {
    falconShooter9.set(0);
    falconShooter35.set(0);
    falconShooter31.set(0);
    falconRear34.set(0);
    falconRear41.set(0);
    neo550Intake15.set(0);
    neo550Rear19.set(0);
    SmartDashboard.putString("MotorTest/Running", "None");
  }
}
