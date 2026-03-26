package frc.robot.commands;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.IOConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Drive.Swerve;
import frc.robot.subsystems.Vision.Limelight;
import java.util.function.BooleanSupplier;

/** Swerve drive command used for teleop period. */
public class AlignAndMove extends Command {
  private Swerve s_Swerve;
  private BooleanSupplier robotCentricSup;
  private CommandXboxController driver;
  private static String name = "limelight-front";
  private int ticks;
  private double epsilon = 0.001;
  private static final double ALIGNMENT_TOLERANCE_DEGREES = 3.0;

  public AlignAndMove(
      Swerve s_Swerve, CommandXboxController driver, BooleanSupplier robotCentricSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    this.driver = driver;
    this.robotCentricSup = robotCentricSup;
    ticks = 0;
  }

  @Override
  public void initialize() {
    s_Swerve.setIsAuto(DriverStation.isAutonomous());
    // Initialize so that the swerve doesn't become grumpy
    s_Swerve.resetModulesToAbsolute();
    Limelight.setPipeline();
    s_Swerve.resetTurnController();
    s_Swerve.setTurnControllerGoal(
        Degree.of(LimelightHelpers.getTX(name) + s_Swerve.getHeadingDegrees()));
  }

  @Override
  public void execute() {
    if (Math.abs(LimelightHelpers.getTX(name)) < epsilon
        && Math.abs(LimelightHelpers.getTY(name)) < epsilon
        && ticks >= 50) {
      Limelight.setPipeline();
      ticks = 0;
    }
    if (LimelightHelpers.getCurrentPipelineIndex(name) == 0.0) {
      Limelight.setPipeline();
    }
    if (!DriverStation.isAutonomous()) {
      s_Swerve.setTurnControllerGoal(
          Degree.of(LimelightHelpers.getTX(name) + s_Swerve.getHeadingDegrees()));
      /* Get Values, Deadband*/
      double[] driverInputs = IOConstants.getDriverInputs(driver.getHID());
      boolean isLimelightAligned = Math.abs(LimelightHelpers.getTX(name)) <= ALIGNMENT_TOLERANCE_DEGREES;
      /* Drive or lock wheels when aligned */
      if (s_Swerve.isFacingTurnTarget() && isLimelightAligned) {
        // Aligned — lock wheels in X position so the robot can't be pushed while shooting
        s_Swerve.lockWheels();
      } else if (isLimelightAligned) {
        s_Swerve.drive(
            new Translation2d(driverInputs[0], driverInputs[1]),
            0.0,
            !robotCentricSup.getAsBoolean(),
            false);
      } else {
        s_Swerve.drive(
            new Translation2d(driverInputs[0], driverInputs[1]),
            s_Swerve.getTurnPidSpeed(),
            !robotCentricSup.getAsBoolean(),
            false);
      }
    }
    ticks++;
  }

  @Override
  public void end(boolean interrupted) {
    s_Swerve.stopDriving();
    LimelightHelpers.setPipelineIndex(name, 0);
    System.out.println("DONE ALIGNINGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG");
  }

  @Override
  public boolean isFinished() {
    // Finish when the swerve heading is within tolerance and the limelight TX is close to zero
    return s_Swerve.isFacingTurnTarget()
        && Math.abs(LimelightHelpers.getTX(name)) <= ALIGNMENT_TOLERANCE_DEGREES;
  }
}
