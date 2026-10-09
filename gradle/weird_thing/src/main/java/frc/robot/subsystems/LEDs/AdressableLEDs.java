// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import static org.wpilib.units.Units.*;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.led.AddressableLED;
import org.wpilib.hardware.led.AddressableLED.ColorOrder;
import org.wpilib.hardware.led.AddressableLEDBuffer;
import org.wpilib.hardware.led.LEDPattern;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.Distance;
import org.wpilib.util.Color;

public class AdressableLEDs extends SubsystemBase {

  private static final int kPort = 0;
  private static final int kLength = 120;

  // Byte order the LED strip expects. WS2812B strips are usually GRB. WPILib 2027 converts our
  // normal RGB colors into this order, so colors below are written as plain RGB.
  // If red and green come out swapped on the robot, change this to ColorOrder.RGB.
  private static final ColorOrder kColorOrder = ColorOrder.GRB;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;

  private static final Distance kLEDSpacing = Meters.of(1.0 / 120.0);

  /** Creates a new AdressableLEDs. */
  public AdressableLEDs() {
    m_led = new AddressableLED(kPort);
    m_buffer = new AddressableLEDBuffer(kLength);

    m_led.setColorOrder(kColorOrder);
    m_led.setLength(kLength);
    // No start() in WPILib 2027: output begins once length and data are set.

    // Default to a scrolling gradient
    setDefaultCommand(runPattern(scrollingFireGradient()).withName("LEDs Idle"));
  }

  // Solid Colors (plain RGB; kColorOrder handles the strip's GRB byte order)

  public LEDPattern solidRed() {
    return LEDPattern.solid(new Color(255.0 / 255.0, 0.0, 0.0));
  }

  public LEDPattern solidGreen() {
    return LEDPattern.solid(new Color(0.0, 255.0 / 255.0, 0.0));
  }

  public LEDPattern solidWhite() {
    return LEDPattern.solid(new Color(255.0 / 255.0, 255.0 / 255.0, 255.0 / 255.0));
  }

  public LEDPattern solidBlack() {
    return LEDPattern.solid(new Color(0.0, 0.0, 0.0));
  }

  public LEDPattern solidPink() {
    // rgb(255, 105, 180)
    return LEDPattern.solid(new Color(255.0 / 255.0, 105.0 / 255.0, 180.0 / 255.0));
  }

  public LEDPattern solidGold() {
    // rgb(255, 215, 0)
    return LEDPattern.solid(new Color(255.0 / 255.0, 215.0 / 255.0, 0.0));
  }

  public LEDPattern solidTeamGold() {
    // #ffa300 = rgb(255, 163, 0)
    return LEDPattern.solid(new Color(255.0 / 255.0, 163.0 / 255.0, 0.0));
  }

  public LEDPattern solidDarkOrange() {
    // rgb(160, 105, 10)
    return LEDPattern.solid(new Color(160.0 / 255.0, 105.0 / 255.0, 10.0 / 255.0));
  }

  // Rainbow
  public LEDPattern scrollingRainbow() {
    LEDPattern base = LEDPattern.rainbow(255, 128);
    return base.scrollAtAbsoluteVelocity(MetersPerSecond.of(1), kLEDSpacing);
  }

  // Fire / Gold Gradient
  public LEDPattern scrollingFireGradient() {
    // Plain RGB colors
    Color yellow = new Color(255.0 / 255.0, 255.0 / 255.0, 0.0);
    Color teamGold = new Color(255.0 / 255.0, 163.0 / 255.0, 0.0);
    Color orange = new Color(255.0 / 255.0, 50.0 / 255.0, 0.0);

    LEDPattern gradient =
        LEDPattern.gradient(LEDPattern.GradientType.CONTINUOUS, yellow, teamGold, orange, teamGold);

    // Scroll from start to end at a nice visible speed
    return gradient.scrollAtAbsoluteVelocity(MetersPerSecond.of(-0.5), kLEDSpacing);
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
