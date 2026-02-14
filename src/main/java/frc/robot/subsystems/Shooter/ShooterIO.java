// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

/** Interface for all shooters to implement */
public interface ShooterIO {
  public static class ShooterIOInputs {
    public double velocityA = 0.0;
    public double velocityB = 0.0;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void setVelocity(double a, double b) {}

  default void stop() {}
}