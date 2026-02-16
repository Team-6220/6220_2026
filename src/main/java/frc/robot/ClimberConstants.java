package frc.robot;

public class ClimberConstants {
  // Motor CAN IDs
  public static final int climberDriverLeftID = 16; // Change to your actual CAN ID
  // TODO: Add right motor CAN ID when installed on full robot
  // public static final int climberDriverRightID = 11; // Change to your actual CAN ID

  // Motor inversions
  public static final boolean motorAInverted = false; // Adjust based on your setup
  // TODO: Add right motor inversion when installed on full robot
  // public static final boolean motorBInverted = true;  // Adjust based on your setup

  // Servo configuration
  public static final int servoPWMPort = 0;

  // Climber height configuration
  // This coefficient converts encoder rotations to height (inches or meters)
  // Example: If 1 rotation = 2 inches of height, coefficient = 2.0
  // Adjust based on your mechanism's gear ratio and drum/pulley diameter
  public static final double climberHeightCoefficient =
      1.2024
          * 3.142; // Distance climber moves per encoder rotation (in inches). This is based on the
  // pitch circumference of a 15T sprocket

  // Encoder offset in inches (adjust to calibrate starting position)
  // e.g., -5.5 means climber starts at -5.5 inches, 0 means starts at 0 inches
  public static final double climberEncoderOffsetInches = 0.0;

  // Stall current threshold for zero-height detection
  // When motor current exceeds this value, the climber has hit bottom
  public static final double STALL_CURRENT_THRESHOLD = 9.0; // Amperes
  public static final double VELOCITY_THRESHOLD = -4000.0;
}
