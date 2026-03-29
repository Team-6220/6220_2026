// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.ArrayList;
import java.util.Optional;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private ArrayList<Command> m_autonomousCommandList;

  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  private final RobotContainer m_robotContainer;

  // Shift tracking for 2026 FRC game
  private double teleOpStartTime = 0.0;
  private int currentShift = 0;
  private double shiftCountdownTime = 0.0;
  private String shiftName = "AUTO";

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    // CameraServer.startAutomaticCapture();
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    
    // Initialize Elastic telemetry client (integrates with DataLogManager and NetworkTables)
    ElasticTelemetry.getInstance();
    System.out.println("[Elastic] Dashboard telemetry client initialized.");
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        Constants.isRed = "red";
      }
      if (ally.get() == Alliance.Blue) {
        Constants.isRed = "blue";
      }
    } else {
      Constants.isRed = "N/A";
    }

    // schedule the autonomous command (example)
    if (m_robotContainer.getAutonomousCommand() != null) {
      CommandScheduler.getInstance().schedule(m_robotContainer.getAutonomousCommand());
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (Constants.isRed.equals("N/A")) {
      Optional<Alliance> ally = DriverStation.getAlliance();
      if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
          Constants.isRed = "red";
        }
        if (ally.get() == Alliance.Blue) {
          Constants.isRed = "blue";
        }
      } else {
        Constants.isRed = "N/A";
      }
    }

    // Track shift during auto for dashboard display
    double matchTime = DriverStation.getMatchTime();
    currentShift = 0;
    shiftName = "AUTO";
    shiftCountdownTime = matchTime; // Counts from 20 seconds down to 0

    // Publish shift info
    SmartDashboard.putString("Match/ShiftName", shiftName);
    SmartDashboard.putNumber("Match/CurrentShift", currentShift);
    SmartDashboard.putNumber("Match/ShiftCountdown", shiftCountdownTime);
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        Constants.isRed = "red";
      }
      if (ally.get() == Alliance.Blue) {
        Constants.isRed = "blue";
      }
    } else {
      Constants.isRed = "N/A";
    }

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Initialize shift tracking
    // Match starts at 2:20 (140s). AUTO 0:20-0:00, TRANSITION 2:20-2:10, SHIFT 1-4, ENDGAME
    // 0:30-0:00
    teleOpStartTime = Timer.getFPGATimestamp();
    currentShift = 0;
    shiftName = "TRANSITION";
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (Constants.isRed.equals("N/A")) {
      Optional<Alliance> ally = DriverStation.getAlliance();
      if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
          Constants.isRed = "red";
        }
        if (ally.get() == Alliance.Blue) {
          Constants.isRed = "blue";
        }
      } else {
        Constants.isRed = "N/A";
      }
    }

    // Update shift tracking based on match time
    double matchTime = DriverStation.getMatchTime();

    if (matchTime >= 130.0) {
      // TRANSITION SHIFT (2:20 - 2:10) = 10 seconds
      currentShift = 0;
      shiftName = "TRANSITION";
      shiftCountdownTime = matchTime - 130.0; // Counts from 0 to 10
    } else if (matchTime >= 105.0) {
      // SHIFT 1 (2:10 - 1:45) = 25 seconds
      currentShift = 1;
      shiftName = "SHIFT 1";
      shiftCountdownTime = matchTime - 105.0; // Counts from 0 to 25
    } else if (matchTime >= 80.0) {
      // SHIFT 2 (1:45 - 1:20) = 25 seconds
      currentShift = 2;
      shiftName = "SHIFT 2";
      shiftCountdownTime = matchTime - 80.0; // Counts from 0 to 25
    } else if (matchTime >= 55.0) {
      // SHIFT 3 (1:20 - 0:55) = 25 seconds
      currentShift = 3;
      shiftName = "SHIFT 3";
      shiftCountdownTime = matchTime - 55.0; // Counts from 0 to 25
    } else if (matchTime >= 30.0) {
      // SHIFT 4 (0:55 - 0:30) = 25 seconds
      currentShift = 4;
      shiftName = "SHIFT 4";
      shiftCountdownTime = matchTime - 30.0; // Counts from 0 to 25
    } else {
      // END GAME (0:30 - 0:00) = 30 seconds
      currentShift = 5;
      shiftName = "END GAME";
      shiftCountdownTime = matchTime; // Counts from 0 to 30
    }

    // Publish shift info to SmartDashboard
    SmartDashboard.putString("Match/ShiftName", shiftName);
    SmartDashboard.putNumber("Match/CurrentShift", currentShift);
    SmartDashboard.putNumber("Match/ShiftCountdown", shiftCountdownTime);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
