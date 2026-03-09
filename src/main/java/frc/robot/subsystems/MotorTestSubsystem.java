package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorTestSubsystem extends SubsystemBase {

  // !! Put your actual CAN IDs here !!
  private final int[] motorIDs = {21, 23, 25, 27, 29, 31}; // Example IDs for drive motors

  private final SparkMax[] motors;
  private int currentMotorIndex = 0;
  private boolean isTesting = false;
  private final Timer testTimer = new Timer();

  private static final double TEST_DURATION = 2.0; // seconds
  private static final double TEST_SPEED = 0.3; // 30% power

  public MotorTestSubsystem() {
    motors = new SparkMax[motorIDs.length];
    SparkMaxConfig config = new SparkMaxConfig();

    for (int i = 0; i < motorIDs.length; i++) {
      motors[i] = new SparkMax(motorIDs[i], MotorType.kBrushless);
      motors[i].configure(
          config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
  }

  public void startNextMotorTest() {
    if (isTesting) return;

    System.out.println("Testing Motor ID: " + motorIDs[currentMotorIndex]);
    motors[currentMotorIndex].set(TEST_SPEED);
    testTimer.reset();
    testTimer.start();
    isTesting = true;
  }

  @Override
  public void periodic() {
    if (isTesting && testTimer.hasElapsed(TEST_DURATION)) {
      motors[currentMotorIndex].set(0);
      isTesting = false;
      currentMotorIndex++;

      if (currentMotorIndex >= motors.length) {
        System.out.println("All motors tested!");
        currentMotorIndex = 0;
      } else {
        System.out.println("Ready for next motor. Press button to continue.");
      }
    }
  }
}
