// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignAndMove;
import frc.robot.commands.ArmToPositionCommand;
import frc.robot.commands.ManualArm;
import frc.robot.commands.SwerveCom;
import frc.robot.commands.TestBeltCommand;
import frc.robot.commands.TestRollerCommand;
import frc.robot.subsystems.Drive.Swerve;
import frc.robot.subsystems.Intake.ArmSubsystem;
import frc.robot.subsystems.Intake.BeltSubsystem;
import frc.robot.subsystems.Intake.RollerSubsystem;
import frc.robot.subsystems.Shooter.AnglerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final AnglerSubsystem m_angler = new AnglerSubsystem();
  private final ArmSubsystem arm = ArmSubsystem.getInstance();
  private final BeltSubsystem belt = BeltSubsystem.getInstance();
  private final RollerSubsystem roller = RollerSubsystem.getInstance();

  private final CommandXboxController m_driverController = new CommandXboxController(0);

  private final Joystick m_joystick = new Joystick(1);

  private final GenericHID m_buttonBoard = new GenericHID(2);

  private static final double ANGLER_MAX_SPEED = 0.15;
  private static final double ANGLER_DEADBAND = 0.1;
  private static final double PRESET_TOP_RPM = 700.0;
  private static final double PRESET_BOTTOM_RPM = 4000.0;

  // !! TUNE THESE before running !!
  // ARM_POSITION_ZERO: resting/home position (usually 0 if arm boots from rest)
  // ARM_POSITION_UP: raised position — start small (e.g. 5) and increase carefully
  private static final double ARM_POSITION_ZERO = 0.0;
  private static final double ARM_POSITION_UP = -10; // <-- TUNE THIS

  public RobotContainer() {
    s_Swerve.configureAutoBuilder();
    s_Swerve.zeroHeading(m_driverController.getHID());

    s_Swerve.setDefaultCommand(
        new SwerveCom(s_Swerve, m_driverController, m_driverController.leftBumper()));

    m_angler.setDefaultCommand(
        Commands.run(
            () -> {
              double input = -m_driverController.getLeftY();
              if (Math.abs(input) < ANGLER_DEADBAND) {
                m_angler.stop();
              } else {
                m_angler.setSpeed(input * ANGLER_MAX_SPEED);
              }
            },
            m_angler));

    //belt.setDefaultCommand(new TestBeltCommand());

    arm.setDefaultCommand(new ManualArm(m_joystick));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    autoChooser.addOption("auto1", new PathPlannerAuto("Auto1"));
    SmartDashboard.putData(autoChooser);
    configureBindings();
  }

  private void configureBindings() {
    Trigger angleUp = new Trigger(() -> m_buttonBoard.getRawButton(5));
    Trigger angleDown = new Trigger(() -> m_buttonBoard.getRawButton(6));
    Trigger shoot = new Trigger(() -> m_buttonBoard.getRawButton(3));
    Trigger intakeIn = new Trigger(() -> m_buttonBoard.getRawButton(1));
    Trigger intakeOut = new Trigger(() -> m_buttonBoard.getRawButton(2));
    Trigger resetArmEncoder = new Trigger (() -> m_buttonBoard.getRawButton(11)); // change later cuz its in shooter branch
    
    resetArmEncoder.onTrue(Commands.runOnce(() -> arm.resetArmEncoder()));
    
    m_driverController
        .y()
        .onTrue(new InstantCommand(() -> s_Swerve.zeroHeading(m_driverController.getHID())));

    m_driverController
        .rightBumper()
        .onTrue(new SwerveCom(s_Swerve, m_driverController, m_driverController.leftBumper()));

    // Arm position commands — a() = home (0), b() = raised position
    // Commands finish automatically when the arm reaches the goal, then hold via maintain()
    m_driverController.a().onTrue(new ArmToPositionCommand(arm, ARM_POSITION_ZERO));
    m_driverController.b().onTrue(new ArmToPositionCommand(arm, ARM_POSITION_UP));

    angleDown.whileTrue(
        Commands.runEnd(() -> m_angler.setSpeed(0.15), () -> m_angler.stop(), m_angler));

    angleUp.whileTrue(
        Commands.runEnd(() -> m_angler.setSpeed(-0.15), () -> m_angler.stop(), m_angler));

    intakeIn.whileTrue(new TestRollerCommand(true));

    intakeOut.whileTrue(new TestRollerCommand(false));

    shoot.whileTrue(
        Commands.parallel(
            Commands.runEnd(
                () -> {
                  m_shooter.setTopGroupVelocityRPS(PRESET_TOP_RPM / 60.0);
                  m_shooter.setBottomGroupVelocityRPS(PRESET_BOTTOM_RPM / 60.0);
                },
                () -> m_shooter.stop())));

    m_driverController
        .leftBumper()
        .whileTrue(
            new AlignAndMove(s_Swerve, m_driverController, m_driverController.rightBumper()));
  }

  public Command getAutonomousCommand() {
    System.out.println("auto: " + autoChooser.getSelected());
    return autoChooser.getSelected();
  }
}
