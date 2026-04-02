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
import frc.robot.commands.ArmToPositionCommand;
import frc.robot.commands.Autos.BasicAutoBlue;
import frc.robot.commands.Autos.BasicAutoRed;
import frc.robot.commands.HashShoot;
import frc.robot.commands.ManualArm;
import frc.robot.commands.PassToAlliance;
import frc.robot.commands.SwerveCom;
import frc.robot.commands.TestRollerCommand;
import frc.robot.subsystems.Drive.Swerve;
import frc.robot.subsystems.Intake.ArmSubsystem;
import frc.robot.subsystems.Intake.BeltSubsystem;
import frc.robot.subsystems.Intake.RollerSubsystem;
import frc.robot.subsystems.LEDs.AdressableLEDs;
import frc.robot.subsystems.Shooter.AnglerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

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
  private final AdressableLEDs s_LED = new AdressableLEDs();
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
  private static final double ALIGN_TOLERANCE_DEG = 2.0;
  private static final double MAX_SHOOT_DISTANCE_M = 4.0;

  public RobotContainer() {
    // Initialize climber subsystem based on robot mode

    // Configure the trigger bindings
    s_Swerve.configureAutoBuilder();
    s_Swerve.zeroHeading(m_driverController.getHID());

    s_Swerve.setDefaultCommand(
        new SwerveCom(s_Swerve, m_driverController, m_driverController.leftBumper()));

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
    Trigger angleUp = new Trigger(() -> m_buttonBoard.getRawButton(15));
    Trigger angleDown = new Trigger(() -> m_buttonBoard.getRawButton(16));
    Trigger intakeOut = new Trigger(() -> m_buttonBoard.getRawButton(2));
    Trigger intakeIn = new Trigger(() -> m_buttonBoard.getRawButton(1));
    Trigger pass = m_driverController.rightBumper();
    Trigger resetEncoder = new Trigger(() -> m_buttonBoard.getRawButton(13));
    Trigger manualArm = new Trigger(() -> m_joystick.getRawButton(1));
    Trigger anglerTest0 = new Trigger(() -> m_buttonBoard.getRawButton(3));
    Trigger arm0 = new Trigger(() -> m_buttonBoard.getRawButton(5));
    Trigger arm90 = new Trigger(() -> m_buttonBoard.getRawButton(6));
    Trigger armReset = new Trigger(() -> m_buttonBoard.getRawButton(14));

    resetEncoder.onTrue(Commands.runOnce(() -> m_angler.resetEncoder()));
    armReset.onTrue(Commands.runOnce(() -> arm.resetEncoder()));

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

    // m_driverController.x().whileTrue(new ShooterTESTER(m_shooter, belt));

    pass.whileTrue(new PassToAlliance(m_angler, m_shooter, belt));

    intakeOut.whileTrue(new TestRollerCommand(true));

    intakeIn.whileTrue(new TestRollerCommand(false));

    m_driverController.rightTrigger().whileTrue(new HashShoot(m_angler, m_shooter, belt));

    m_driverController
        .leftTrigger()
        .whileTrue(
            new AlignAndMove(s_Swerve, m_driverController, m_driverController.rightBumper()));

    arm0.onTrue(new ArmToPositionCommand(arm, -2));

    arm90.onTrue(new ArmToPositionCommand(arm, -29.785476684570312));

    // ==================== State-Based LED Controls ====================

    // --- Individual state triggers ---
    Trigger flywheelsReady =
        new Trigger(() -> m_shooter.isAtSpeedFly(m_shooter.getTopTargetRPM() / 60.0));

    Trigger aligned =
        new Trigger(
            () ->
                Math.abs(LimelightHelpers.getTX("limelight-front")) < ALIGN_TOLERANCE_DEG
                    && LimelightHelpers.getTV("limelight-front"));

    Trigger inRange =
        new Trigger(
            () -> {
              double dist = m_shooter.getDist();
              return dist > 0 && dist <= MAX_SHOOT_DISTANCE_M;
            });

    // Stable lock for driver rumble to avoid flicker/noise
    Trigger targetLocked = aligned.and(inRange).debounce(0.1);

    // --- Combined "ready to shoot" trigger ---
    // We require everything (speed, alignment, range) for the final triggers so rumble corresponds
    // to the actual firing window.
    Trigger readyToShoot = flywheelsReady.and(aligned).and(inRange);

    // --- LED bindings (lowest to highest priority) ---

    // Flywheels spinning but NOT ready -> fast blink red
    flywheelsReady
        .negate()
        .and(new Trigger(() -> m_shooter.getMotor9RPM() > 100))
        .whileTrue(s_LED.runPattern(s_LED.blink(s_LED.solidRed(), 0.15)));

    // Flywheels at speed but not aligned or out of range -> solid dark orange
    flywheelsReady
        .and(readyToShoot.negate())
        .whileTrue(s_LED.runPattern(s_LED.blink(s_LED.solidDarkOrange(), 0.15)));

    aligned.whileTrue(s_LED.runPattern(s_LED.solidGreen()));

    // Haptic feedback (rumble): Pulse when target is aligned + in range (before full spin-up).
    aligned.onTrue(
        Commands.runOnce(
                () -> {
                  m_driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
                })
            .andThen(Commands.waitSeconds(0.25))
            .andThen(
                Commands.runOnce(
                    () -> {
                      m_driverController.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
                    })));
  }

  /** Publish shooter booleans needed by Elastic dashboard widgets. */
  public void publishDriverDashboardBooleans() {
    boolean aligned =
        LimelightHelpers.getTV("limelight-front")
            && Math.abs(LimelightHelpers.getTX("limelight-front")) < ALIGN_TOLERANCE_DEG;
    double dist = m_shooter.getDist();
    boolean shortRange = dist > 0 && dist <= MAX_SHOOT_DISTANCE_M;

    SmartDashboard.putBoolean("Shooter/Aligned", aligned);
    SmartDashboard.putBoolean("Shooter/ShortRange", shortRange);
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
