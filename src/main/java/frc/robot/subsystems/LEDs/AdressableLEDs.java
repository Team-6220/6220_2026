// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AdressableLEDs extends SubsystemBase {

  // Change kPort to what is necessary and make kLength the number of LEDs on the strip
  private static final int kPort = 9;
  private static final int kLength = 120;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  private static final Distance kLEDSpacing = Meters.of(1.0 / 120.0);

  /** Creates a new AdressableLEDs. */
  public AdressableLEDs() {
    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);

    m_led.setLength(kLength);
    m_led.start();

    // Default to a scrolling gradient
    setDefaultCommand(runPattern(scrollingFireGradient()).withName("LEDs Idle"));
  }

  // Solid Colors

  public LEDPattern solidRed() {
    return LEDPattern.solid(new Color(0.0, 255.0 / 255.0, 0.0));
  }

  public LEDPattern solidGreen() {
    return LEDPattern.solid(new Color(255.0 / 255.0, 0.0, 0.0));
  }

  public LEDPattern solidWhite() {
    return LEDPattern.solid(new Color(255.0 / 255.0, 255.0 / 255.0, 255.0 / 255.0));
  }

  public LEDPattern solidBlack() {
    return LEDPattern.solid(new Color(0.0, 0.0, 0.0));
  }

  public LEDPattern solidPink() {
    return LEDPattern.solid(new Color(105.0 / 255.0, 255.0 / 255.0, 180.0 / 255.0));
  }

  public LEDPattern solidGold() {
    return LEDPattern.solid(new Color(215.0 / 255.0, 255.0 / 255.0, 0.0));
  }

  public LEDPattern solidTeamGold() {
    // #ffa300 = rgb(255, 163, 0)
    // Swapped R and G for GRB format
    return LEDPattern.solid(new Color(163.0 / 255.0, 255.0 / 255.0, 0.0));
  }

  public LEDPattern solidDarkOrange() {
    // Swapped R and G for GRB format
    return LEDPattern.solid(new Color(140.0 / 255.0, 255.0 / 255.0, 0.0));
  }

  // Rainbow
  public LEDPattern scrollingRainbow() {
    LEDPattern base = LEDPattern.rainbow(255, 128);
    return base.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLEDSpacing);
  }

  // Fire / Gold Gradient
  public LEDPattern scrollingFireGradient() {
    // GRB formatted colors
    Color yellow = new Color(255.0 / 255.0, 255.0 / 255.0, 0.0);
    Color teamGold = new Color(163.0 / 255.0, 255.0 / 255.0, 0.0);
    Color orange = new Color(50.0 / 255.0, 255.0 / 255.0, 0.0);

    LEDPattern gradient =
        LEDPattern.gradient(
            LEDPattern.GradientType.kContinuous, yellow, teamGold, orange, teamGold);

    // Scroll from start to end at a nice visible speed
    return gradient.scrollAtAbsoluteSpeed(MetersPerSecond.of(-0.5), kLEDSpacing);
  }

  // Modifiers

  public LEDPattern blink(LEDPattern base, double seconds) {
    return base.blink(Seconds.of(seconds));
  }

  public LEDPattern breathe(LEDPattern base, double seconds) {
    return base.breathe(Seconds.of(seconds));
  }

  public LEDPattern brightness(LEDPattern base, double percent) {
    return base.atBrightness(Percent.of(percent));
  }

  public LEDPattern reverse(LEDPattern base) {
    return base.reversed();
  }

  public LEDPattern syncedBlink(LEDPattern base) {
    return base.synchronizedBlink(RobotController::getRSLState);
  }

  // Command Helper
  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(m_buffer));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_led.setData(m_buffer);
  }
}
