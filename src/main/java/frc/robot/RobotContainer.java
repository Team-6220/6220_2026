// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
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
import frc.robot.commands.Autos.BasicAutoBlue;
import frc.robot.commands.Autos.BasicAutoRed;
import frc.robot.commands.HashShoot;
import frc.robot.commands.ManualArm;
import frc.robot.commands.PassToAlliance;
import frc.robot.commands.ShooterTESTER;
import frc.robot.commands.SwerveCom;
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

    // Prespin flywheels when limelight sees a tag
    // m_shooter.setDefaultCommand(
    //     Commands.run(
    //         () -> {
    //           if (m_shooter.getDist() > 0) {
    //             m_shooter.setTopGroupVelocityRPS(m_shooter.getTopTargetRPM() / 60.0);
    //           } else {
    //             m_shooter.stop();
    //           }
    //         },
    //         m_shooter));

    // m_angler.setDefaultCommand(
    //     Commands.run(
    //         () -> {
    //           double input = -m_driverController.getLeftY(); // negative so up = up
    //           if (Math.abs(input) < ANGLER_DEADBAND) {
    //             m_angler.stop();
    //           } else {
    //             m_angler.setSpeed(input * ANGLER_MAX_SPEED);
    //           }
    //         },
    //         m_angler));

    // belt.setDefaultCommand(new TestBeltCommand());

    arm.setDefaultCommand(new ManualArm(m_joystick));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // TODO: Register named commands as needed for auto
    // NamedCommands.registerCommand("AutoClimber", new AutoClimberCommand(climberSubsystem));

    // NamedCommands.registerCommand(null, null);
    autoChooser.addOption("Red", new BasicAutoRed(s_Swerve, m_angler, m_shooter, belt));
    autoChooser.addOption("Blue", new BasicAutoBlue(s_Swerve, m_angler, m_shooter, belt));
    SmartDashboard.putData(autoChooser);
    configureBindings();
  }

  private void configureBindings() {
    Trigger angleUp = new Trigger(() -> m_buttonBoard.getRawButton(5));
    Trigger angleDown = new Trigger(() -> m_buttonBoard.getRawButton(6));
    Trigger intakeIn = new Trigger(() -> m_buttonBoard.getRawButton(2));
    Trigger intakeOut = new Trigger(() -> m_buttonBoard.getRawButton(1));
    Trigger pass = new Trigger(() -> m_buttonBoard.getRawButton(4));
    Trigger resetEncoder = new Trigger(() -> m_buttonBoard.getRawButton(3));
    Trigger manualArm = new Trigger(() -> m_joystick.getRawButton(1));
    resetEncoder.onTrue(Commands.runOnce(() -> m_angler.resetEncoder()));

    // Test buttons for angler PID: button 4 -> set to 1.0,
    Trigger anglerTestOne = new Trigger(() -> m_buttonBoard.getRawButton(4));
    Trigger anglerTest0 = new Trigger(() -> m_buttonBoard.getRawButton(11));

    anglerTestOne.onTrue(
        Commands.run(() -> m_angler.setAngle(m_angler.getAnglerAngle()), m_angler)
            .until(() -> m_angler.isAtAngle(m_angler.getAnglerAngle())));

    anglerTest0.onTrue(
        Commands.run(() -> m_angler.setAngle(0), m_angler).until(() -> m_angler.isAtAngle(0)));

    m_driverController
        .y()
        .onTrue(new InstantCommand(() -> s_Swerve.zeroHeading(m_driverController.getHID())));

    angleDown.whileTrue(
        Commands.runEnd(() -> m_angler.setSpeed(-0.15), () -> m_angler.stop(), m_angler));
    angleUp.whileTrue(
        Commands.runEnd(() -> m_angler.setSpeed(0.15), () -> m_angler.stop(), m_angler));
    manualArm.onTrue(new ManualArm(m_joystick));

    m_driverController
        .a()
        .onTrue(
            Commands.run(() -> m_angler.setAngle(0), m_angler).until(() -> m_angler.isAtAngle(0)));

    // Arm position commands — a() = home (0), b() = raised position
    // Commands finish automatically when the arm reaches the goal, then hold via maintain()
    m_driverController.a().onTrue(new ArmToPositionCommand(arm, ARM_POSITION_ZERO));
    m_driverController.b().onTrue(new ArmToPositionCommand(arm, ARM_POSITION_UP));

    angleDown.whileTrue(
        Commands.runEnd(() -> m_angler.setSpeed(0.15), () -> m_angler.stop(), m_angler));
    m_driverController
        .b()
        .whileTrue(Commands.runEnd(() -> m_angler.setSpeed(0.15), () -> m_angler.stop(), m_angler));

    m_driverController.x().whileTrue(new ShooterTESTER(m_shooter, belt));

    m_driverController.rightBumper().whileTrue(new PassToAlliance(m_angler, m_shooter, belt));

    intakeIn.whileTrue(new TestRollerCommand(true));

    intakeOut.whileTrue(new TestRollerCommand(false));

    m_driverController.rightTrigger().whileTrue(new HashShoot(m_angler, m_shooter, belt));

    m_driverController
        .leftTrigger()
        .whileTrue(
            new AlignAndMove(s_Swerve, m_driverController, m_driverController.rightBumper()));

    // ========== SysId Buttons (button board) ==========
    // Button 8: Quasistatic Forward (slow ramp up)
    // Button 9: Quasistatic Reverse (slow ramp down)
    // Button 10: Dynamic Forward (step voltage)
    // Button 11 is already used for anglerTest0, so using button 12 instead
    // Button 12: Dynamic Reverse (step voltage reverse)
    // new Trigger(() -> m_joystick.getRawButton(3))
    //     .whileTrue(m_shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // new Trigger(() -> m_joystick.getRawButton(4))
    //     .whileTrue(m_shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // new Trigger(() -> m_joystick.getRawButton(5))
    //     .whileTrue(m_shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // new Trigger(() -> m_joystick.getRawButton(6))
    //     .whileTrue(m_shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    System.out.println("auto: " + autoChooser.getSelected());
    return autoChooser.getSelected();
  }
}
