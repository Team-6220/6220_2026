// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

/**
 * Shooter IO abstraction.
 *
 * <p>This interface is responsible for reading sensor inputs and controlling the
 * hardware for the shooter subsystem. Implementations of this interface should
 * handle all direct interaction with motors, encoders, etc.
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
}
