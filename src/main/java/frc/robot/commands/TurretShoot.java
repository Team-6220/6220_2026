// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake.BeltSubsystem;
import frc.robot.subsystems.Shooter.AnglerSubsystem;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

/**
 * TurretShoot: like HashShoot but waits until turret, angler, and flywheels are at setpoint before
 * actuating the belt to feed balls.
 */
public class TurretShoot extends Command {
  AnglerSubsystem m_angler;

  ShooterSubsystem m_shoot;

  BeltSubsystem m_belt;
  TurretSubsystem m_turret;
  CommandXboxController controller;
  double degrees;
  double rpm;
  Distance dist;

  public TurretShoot(
      AnglerSubsystem m_angler,
      ShooterSubsystem m_shoot,
      BeltSubsystem m_belt,
      TurretSubsystem m_turret,
      CommandXboxController controller) {
    this.m_angler = m_angler;
    this.m_shoot = m_shoot;
    this.m_belt = m_belt;
    this.m_turret = m_turret;
    this.controller = controller;
    addRequirements(m_angler, m_shoot, m_belt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
      dist = m_shoot.getDist();
      Double[] vals = ShooterConstants.rpmAngle.get(dist);
      rpm = vals[0];
      degrees = vals[1];
    } catch (Exception e) {
      System.err.println("Error occurred while initializing TurretShoot command.");
      Double[] vals = ShooterConstants.rpmAngle.get(Meters.of(0.0));
      rpm = vals[0];
      degrees = vals[1];
    }
    // Reset shooter state for new shot sequence (enables first shot boost)
    m_shoot.resetShootingState();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Always drive angler and request shooter velocity
    m_angler.setAngle(degrees);
    m_shoot.runAtTargetVelocity(rpm, controller);

    // Only feed balls when turret, angler and flywheels are at their setpoints
    boolean turretReady = m_turret.atSetpoint();
    boolean anglerReady = m_angler.isAtAngle(degrees);
    boolean flyReady = m_shoot.isAtSpeedFly(rpm / 60.0);

    if (turretReady && anglerReady && flyReady) {
      // Feed slowly when ready
      m_belt.simpleDrive(-0.5);
    } else {
      // Stop feeding until everything is ready
      m_belt.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_angler.stop();
    m_shoot.stop();
    m_belt.stop();
    new InstantCommand(() -> m_angler.setAngle(0), m_angler).until(() -> m_angler.isAtAngle(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
