package frc.robot;

public class ClimberConstants {
  // Motor CAN IDs
  public static final int climberDriverLeftID = 16;
  // TODO: Add right motor CAN ID when installed on full robot
  // public static final int climberDriverRightID = 11;

  // Motor inversions
  public static final boolean leftMotorInverted = true;
  // TODO: Add right motor inversion when installed on full robot
  // public static final boolean rightMotorInverted = true; 
  // Servo configuration
  public static final int leftServoPWMPort = 0;
  public static final int rightServoPWMPort = 1;

  // Stall current threshold for zero-height detection (bottom limit)
  // When motor current exceeds this value, the climber has hit bottom
  public static final double STALL_CURRENT_THRESHOLD_BOTTOM = 8.0; // Amperes / NOTE: THIS WILL ONLY BE USED WHEN TUNING MODE in Constants.java is FALSE

  // Stall current threshold for max-height detection (top limit)
  // When motor current exceeds this value, the climber has hit top
  public static final double STALL_CURRENT_THRESHOLD_TOP = 100.0; // Amperes / NOTE: THIS WILL ONLY BE USED WHEN TUNING MODE in Constants.java is FALSE

  // Velocity threshold for stall detection (only used for bottom)
  // Motor must be running at this velocity to distinguish stall from acceleration
  public static final double VELOCITY_THRESHOLD = 4000.0;

  // Motor speed configuration
  // Speed for climber movement (0.0 to 1.0)
  public static final double CLIMBER_SPEED = 0.75;
}
