/**
 * Swerve drive command used for teleop period.
 */
package frc.robot.commands;

import frc.robot.Constants.OIConstants;
// import frc.robot.Constants.SwerveConstants;
// import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private BooleanSupplier robotCentricSup;
    private CommandXboxController driver;

    public TeleopSwerve(Swerve s_Swerve, CommandXboxController driver, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.driver = driver;
        this.robotCentricSup = robotCentricSup;

    }

    @Override
    public void initialize() {
        s_Swerve.setIsAuto(false);
        // Initilize so that the swerve doesn't become grumpy
        s_Swerve.resetModulesToAbsolute();
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double[] driverInputs = OIConstants.getDriverInputs(driver.getHID());
        /* Drive */
        s_Swerve.drive(
            new Translation2d(driverInputs[0],driverInputs[1]),
            driverInputs[2], 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}