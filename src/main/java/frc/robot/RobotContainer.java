// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  // Controllers
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);

  // Shooter preset speeds (adjust these based on testing)
  private static final double SPEAKER_SHOT_RPM = 4000.0;
  private static final double AMP_SHOT_RPM = 2000.0;
  private static final double FEED_SHOT_RPM = 3000.0;
  private static final double IDLE_RPM = 1500.0;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // ==================== OPERATOR CONTROLS ====================
    
    // Right Trigger - Run shooter at speaker speed
    m_operatorController.rightTrigger()
        .whileTrue(new ShooterCommand(m_shooterSubsystem, SPEAKER_SHOT_RPM))
        .onFalse(new ShooterCommand.Stop(m_shooterSubsystem));

    // Left Trigger - Run shooter at amp speed
    m_operatorController.leftTrigger()
        .whileTrue(new ShooterCommand(m_shooterSubsystem, AMP_SHOT_RPM))
        .onFalse(new ShooterCommand.Stop(m_shooterSubsystem));

    // Right Bumper - Run shooter at feed speed
    m_operatorController.rightBumper()
        .whileTrue(new ShooterCommand(m_shooterSubsystem, FEED_SHOT_RPM))
        .onFalse(new ShooterCommand.Stop(m_shooterSubsystem));

    // Left Bumper - Run shooter at idle speed (for quick shots)
    m_operatorController.leftBumper()
        .whileTrue(new ShooterCommand(m_shooterSubsystem, IDLE_RPM))
        .onFalse(new ShooterCommand.Stop(m_shooterSubsystem));

    // B Button - Emergency stop shooter
    m_operatorController.b()
        .onTrue(new ShooterCommand.Stop(m_shooterSubsystem));

    // ==================== SHOOTER SELECTION (D-PAD) ====================
    
    // D-Pad Up - Use all 3 shooters
    m_operatorController.povUp()
        .onTrue(Commands.runOnce(() -> 
          m_shooterSubsystem.setActiveShooters(ShooterSubsystem.ShooterSelection.ALL)));

    // D-Pad Left - Use left shooter only (Shooter 1)
    m_operatorController.povLeft()
        .onTrue(Commands.runOnce(() -> 
          m_shooterSubsystem.setActiveShooters(ShooterSubsystem.ShooterSelection.SHOOTER_1)));

    // D-Pad Down - Use center shooter only (Shooter 2)
    m_operatorController.povDown()
        .onTrue(Commands.runOnce(() -> 
          m_shooterSubsystem.setActiveShooters(ShooterSubsystem.ShooterSelection.SHOOTER_2)));

    // D-Pad Right - Use right shooter only (Shooter 3)
    m_operatorController.povRight()
        .onTrue(Commands.runOnce(() -> 
          m_shooterSubsystem.setActiveShooters(ShooterSubsystem.ShooterSelection.SHOOTER_3)));

    // A Button - Use left + center shooters
    m_operatorController.a()
        .onTrue(Commands.runOnce(() -> 
          m_shooterSubsystem.setActiveShooters(ShooterSubsystem.ShooterSelection.SHOOTERS_1_2)));

    // X Button - Use center + right shooters  
    m_operatorController.x()
        .onTrue(Commands.runOnce(() -> 
          m_shooterSubsystem.setActiveShooters(ShooterSubsystem.ShooterSelection.SHOOTERS_2_3)));

    // Y Button - Use left + right shooters (skip center)
    m_operatorController.y()
        .onTrue(Commands.runOnce(() -> 
          m_shooterSubsystem.setActiveShooters(ShooterSubsystem.ShooterSelection.SHOOTERS_1_3)));

    // ==================== DRIVER CONTROLS (OPTIONAL) ====================
    
    // Driver can also quickly prep all shooters for speaker
    m_driverController.rightBumper()
        .whileTrue(new ShooterCommand(m_shooterSubsystem, SPEAKER_SHOT_RPM));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Commands.sequence(
        // Spin up shooter
        new ShooterCommand(m_shooterSubsystem, SPEAKER_SHOT_RPM, true),
        // Wait for shooter to be at speed
        Commands.waitSeconds(0.5),
        // You would add your shoot sequence here
        Commands.print("Ready to shoot!"),
        // Stop shooter at end
        new ShooterCommand.Stop(m_shooterSubsystem)
    );
  }

  /**
   * Gets the shooter subsystem
   * @return The shooter subsystem instance
   */
  public ShooterSubsystem getShooterSubsystem() {
    return m_shooterSubsystem;
  }

  /**
   * Creates a command to shoot at a specific RPM
   * @param rpm Target velocity in RPM
   * @return Command to run shooter at specified RPM
   */
  public Command shootAtRPM(double rpm) {
    return new ShooterCommand(m_shooterSubsystem, rpm);
  }

  /**
   * Creates a command to shoot at speaker
   * @return Command to run shooter at speaker speed
   */
  public Command shootSpeaker() {
    return new ShooterCommand(m_shooterSubsystem, SPEAKER_SHOT_RPM);
  }

  /**
   * Creates a command to shoot at amp
   * @return Command to run shooter at amp speed
   */
  public Command shootAmp() {
    return new ShooterCommand(m_shooterSubsystem, AMP_SHOT_RPM);
  }

  /**
   * Creates a command to prepare shooter (spin up and wait for ready)
   * @param rpm Target RPM
   * @return Command that finishes when shooter is at speed
   */
  public Command prepareShooter(double rpm) {
    return new ShooterCommand(m_shooterSubsystem, rpm, true);
  }
}