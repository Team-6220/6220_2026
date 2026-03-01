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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignHub;
import frc.robot.commands.DigitalClimberCommand;
import frc.robot.commands.SwerveCom;
import frc.robot.subsystems.Climber.ClimberIOReal;
import frc.robot.subsystems.Climber.ClimberIOSim;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Drive.Swerve;
import frc.robot.subsystems.Vision.Limelight;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final SendableChooser<Command> autoChooser;

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final ClimberSubsystem climberSubsystem;

  private final Limelight s_Limelight = new Limelight();

  private final CommandXboxController m_driverController = new CommandXboxController(0);

  private final Joystick m_joystick = new Joystick(1);

  private final GenericHID m_buttonBoard = new GenericHID(2);

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

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // TODO: Register named commands as needed for auto
    // NamedCommands.registerCommand("AutoClimber", new AutoClimberCommand(climberSubsystem));

    // NamedCommands.registerCommand(null, null);
    autoChooser.addOption("auto1", new PathPlannerAuto("Auto1"));
    SmartDashboard.putData(autoChooser);
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    m_driverController
        .y()
        .onTrue(new InstantCommand(() -> s_Swerve.zeroHeading(m_driverController.getHID())));

    m_driverController
        .rightBumper()
        .onTrue(new SwerveCom(s_Swerve, m_driverController, m_driverController.leftBumper()));

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

    m_driverController.leftBumper().whileTrue(new AlignHub());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    System.out.println("auto: " + autoChooser.getSelected());
    return autoChooser.getSelected();
  }
  // An example command will be run in autonomous

}
