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
import frc.robot.commands.GoToZeroCommand;
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
        new SwerveCom(
            s_Swerve, m_driverController, () -> m_driverController.getHID().getLeftBumperButton()));

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

    // Climber servo control - A button toggles servo between deployed (1.0) and retracted (0.0)
    m_driverController
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  double currentPosition = climberSubsystem.getServoPosition();
                  if (currentPosition > 0.5) {
                    // Servo is deployed, retract it
                    climberSubsystem.setServoPosition(0.0);
                  } else {
                    // Servo is retracted, deploy it
                    climberSubsystem.setServoPosition(1.0);
                  }
                }));

    // Climber control mode toggle - B button toggles between LEFT, RIGHT, and BOTH
    m_driverController.b().onTrue(new InstantCommand(climberSubsystem::toggleControlMode));

    // Climber motor control - continuously read LT and RT analog trigger values
    // RT: drives up (positive, 0 to 0.75)
    // LT: drives down (negative, 0 to -0.75)
    new Trigger(
            () -> {
              double leftTrigger = m_driverController.getLeftTriggerAxis();
              double rightTrigger = m_driverController.getRightTriggerAxis();
              // Either trigger is pressed
              return leftTrigger > 0.05 || rightTrigger > 0.05;
            })
        .whileTrue(
            new edu.wpi.first.wpilibj2.command.Command() {
              {
                addRequirements(climberSubsystem);
              }

              @Override
              public void execute() {
                double leftTrigger = m_driverController.getLeftTriggerAxis();
                double rightTrigger = m_driverController.getRightTriggerAxis();
                // RT (0-1) maps to positive speed (0 to 0.75)
                // LT (0-1) maps to negative speed (0 to -0.75)
                double speed = (rightTrigger * 0.75) - (leftTrigger * 0.75);
                climberSubsystem.setMotorSpeed(speed);
              }

              @Override
              public void end(boolean interrupted) {
                climberSubsystem.stop();
              }

              @Override
              public boolean isFinished() {
                return false;
              }
            }.withName("AnalogClimberControl"));

    // Climber height control - X button zeros climber to bottom using stall detection
    m_driverController.x().whileTrue(new GoToZeroCommand(climberSubsystem));

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
