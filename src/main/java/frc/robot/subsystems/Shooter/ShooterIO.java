// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

<<<<<<< HEAD
/** Constants for shooter motor CAN IDs */
public class ShooterIO {
  // Update these to match your actual CAN IDs
  public static final int KICKER_1_ID = 20;
  public static final int KICKER_2_ID = 21;
  public static final int MOTOR_1_ID = 10;
  public static final int MOTOR_2_ID = 11;
  public static final int MOTOR_3_ID = 12;
  public static final int HOOD_ID = 30;
=======
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
    // Add sensor fields here (e.g., velocities, currents, temperatures)
  }

  /**
   * Updates the given {@link Inputs} instance with the latest sensor readings.
   *
   * @param inputs the container to populate with current sensor values
   */
  void updateInputs(Inputs inputs);
>>>>>>> f422a4876a208394cee7d81b8036836c1a873274
}

