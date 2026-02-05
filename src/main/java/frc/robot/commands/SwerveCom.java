package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.OIConstants;
import frc.robot.subsystems.Drive.Swerve;
import java.util.function.BooleanSupplier;

/** Swerve drive command used for teleop period. */
public class SwerveCom extends Command {
  private Swerve s_Swerve;
  private BooleanSupplier robotCentricSup;
  private CommandXboxController driver;

  public SwerveCom(
      Swerve s_Swerve, CommandXboxController driver, BooleanSupplier robotCentricSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    this.driver = driver;
    this.robotCentricSup = robotCentricSup;
  }
  public SwerveCom(
      Swerve s_Swerve, CommandXboxController driver, BooleanSupplier robotCentricSup, ) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    this.driver = driver;
    this.robotCentricSup = robotCentricSup;
  }

  @Override
  public void initialize() {
    s_Swerve.setIsAuto(DriverStation.isAutonomous());
    // Initilize so that the swerve doesn't become grumpy
    s_Swerve.resetModulesToAbsolute();
  }
  

  @Override
  public void execute() {
    /* Get Values, Deadband*/
    double[] driverInputs = OIConstants.getDriverInputs(driver.getHID());
    /* Drive */
    if(DriverStation.isAutonomous()){
        s_Swerve.drive(
        new Translation2d(-driverInputs[0], -driverInputs[1]),
        -driverInputs[2],
        false,
        true);
    }
    else{
    s_Swerve.drive(
        new Translation2d(-driverInputs[0], -driverInputs[1]),
        -driverInputs[2],
        !robotCentricSup.getAsBoolean(),
        true);}
  }
}
