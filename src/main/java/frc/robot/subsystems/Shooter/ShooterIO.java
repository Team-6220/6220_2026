// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

/**
 * Shooter IO abstraction.
 *
 * <p>This interface is responsible for reading sensor inputs and controlling the hardware for the
 * shooter subsystem. Implementations of this interface should handle all direct interaction with
 * motors, encoders, etc.
 */
public interface ShooterIO {
  /** Container class for all shooter sensor inputs. */
  public static class Inputs {
    public double motor41VelocityRPS = 0.0;
    public double motor1VelocityRPS = 0.0;
    public double motor9VelocityRPS = 0.0;
    public double motor31VelocityRPS = 0.0;
    public double motor2VelocityRPS = 0.0;

    public double motor41Voltage = 0.0;
    public double motor1Voltage = 0.0;
    public double motor9Voltage = 0.0;
    public double motor31Voltage = 0.0;
    public double motor2Voltage = 0.0;

    public double motor41Current = 0.0;
    public double motor1Current = 0.0;
    public double motor9Current = 0.0;
    public double motor31Current = 0.0;
    public double motor2Current = 0.0;
  }

  /**
   * Updates the given {@link Inputs} instance with the latest sensor readings.
   *
   * @param inputs the container to populate with current sensor values
   */
  public default void updateInputs(Inputs inputs) {}

  /** Sets all shooter motors to the given velocity in RPS. */
  public default void setVelocityRPS(double rps) {}

  /** Stops all shooter motors. */
  public default void stop() {}

  /** Stops motors 41 and 1 (top group). */
  public default void stopTopGroup() {}

  /** Stops motors 9, 2, and 31 (bottom group). */
  public default void stopBottomGroup() {}

  /** Sets all shooter motors to a percentage of max output. */
  public default void setPercentOutput(double percent) {}

  /** Sets motors 41 and 1 (top group) to a percentage of max output. */
  public default void setTopGroupPercent(double percent) {}

  /** Sets motors 9, 2, and 31 (bottom group) to a percentage of max output. */
  public default void setBottomGroupPercent(double percent) {}

  /**
   * Applies PID and feedforward gains to all motors.
   *
   * @param kP proportional gain
   * @param kI integral gain
   * @param kD derivative gain
   * @param kV velocity feedforward
   * @param kS static feedforward
   * @param kA acceleration feedforward
   */
  public default void applyPIDConfigs(
      double kP, double kI, double kD, double kV, double kS, double kA) {}

  /**
   * Sets the neutral mode for all motors.
   *
   * @param brake true for brake mode, false for coast mode
   */
  public default void setNeutralMode(boolean brake) {}
}
