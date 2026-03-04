// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlignHub;
import frc.robot.commands.DigitalClimberCommand;
import frc.robot.commands.SwerveCom;
import frc.robot.subsystems.Climber.ClimberIOReal;
import frc.robot.subsystems.Climber.ClimberIOSim;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Drive.Swerve;
import frc.robot.subsystems.Shooter.AnglerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Vision.Limelight;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final AnglerSubsystem m_angler = new AnglerSubsystem();

  // Controller
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  private final Joystick m_joystick = new Joystick(1);

  private final GenericHID m_buttonBoard = new GenericHID(2);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final ClimberSubsystem climberSubsystem;

  private final Limelight s_Limelight = new Limelight();

  // Max angler speed (keep it slow)
  private static final double ANGLER_MAX_SPEED = 0.15;
  private static final double ANGLER_DEADBAND = 0.1;

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // Initialize climber subsystem based on robot mode
    if (RobotBase.isSimulation()) {
      climberSubsystem = new ClimberSubsystem(new ClimberIOSim());
    } else {
      climberSubsystem = new ClimberSubsystem(new ClimberIOReal());
    }

    // Configure the trigger bindings
    s_Swerve.configureAutoBuilder();
    s_Swerve.zeroHeading(m_driverController.getHID());

    s_Swerve.setDefaultCommand(
        new SwerveCom(s_Swerve, m_driverController, m_driverController.leftBumper()));

    // ==================== BUTTON BOARD CLIMBER BINDINGS ====================
    final int LEFT_RESET_BUTTON = 1;
    final int RIGHT_RESET_BUTTON = 2;
    final int LEFT_UP_BUTTON = 3;
    final int RIGHT_UP_BUTTON = 4;
    final int LEFT_DOWN_BUTTON = 5;
    final int RIGHT_DOWN_BUTTON = 6;
    final int LEFT_SERVO_TOGGLE_BUTTON = 7;
    final int RIGHT_SERVO_TOGGLE_BUTTON = 8;

    // Set climber control as the default command - runs continuously
    // This allows the command to detect buttons even if they're held before robot enable
    climberSubsystem.setDefaultCommand(
        new DigitalClimberCommand(
            climberSubsystem,
            m_buttonBoard,
            LEFT_RESET_BUTTON,
            RIGHT_RESET_BUTTON,
            LEFT_UP_BUTTON,
            RIGHT_UP_BUTTON,
            LEFT_DOWN_BUTTON,
            RIGHT_DOWN_BUTTON,
            LEFT_SERVO_TOGGLE_BUTTON,
            RIGHT_SERVO_TOGGLE_BUTTON));

    // Left joystick Y: control angler up/down (slow)
    m_angler.setDefaultCommand(
        Commands.run(
            () -> {
              double input = -m_driverController.getLeftY(); // negative so up = up
              if (Math.abs(input) < ANGLER_DEADBAND) {
                m_angler.stop();
              } else {
                m_angler.setSpeed(input * ANGLER_MAX_SPEED);
              }
            },
            m_angler));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // TODO: Register named commands as needed for auto
    // NamedCommands.registerCommand("AutoClimber", new AutoClimberCommand(climberSubsystem));

    // NamedCommands.registerCommand(null, null);
    autoChooser.addOption("auto1", new PathPlannerAuto("Auto1"));
    SmartDashboard.putData(autoChooser);
    configureBindings();
  }

  private void configureBindings() {
    m_driverController
        .y()
        .onTrue(new InstantCommand(() -> s_Swerve.zeroHeading(m_driverController.getHID())));
    // Left bumper: hold to spin motors 9, 31
    m_driverController
        .leftBumper()
        .whileTrue(
            Commands.runEnd(
                () -> m_shooter.setBottomGroupPercent(0.8), () -> m_shooter.stopBottomGroup()));

    // Right bumper: hold to spin motors 41, 1
    m_driverController
        .rightBumper()
        .whileTrue(
            Commands.runEnd(
                () -> m_shooter.setTopGroupPercent(0.5), () -> m_shooter.stopTopGroup()));

    // A button: hold to test angler motor
    m_driverController
        .a()
        .whileTrue(Commands.runEnd(() -> m_angler.setSpeed(0.15), () -> m_angler.stop()));
    m_driverController
        .b()
        .whileTrue(Commands.runEnd(() -> m_angler.setSpeed(-0.15), () -> m_angler.stop()));
    m_driverController.leftBumper().whileTrue(new AlignHub());
  }

  public Command getAutonomousCommand() {
    System.out.println("auto: " + autoChooser.getSelected());
    return autoChooser.getSelected();
  }
}
