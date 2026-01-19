package frc.lib.util;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.smartdashboard.*;

public class TunableNumber {
  private static final String TABLE_KEY = "TunableNumbers";

  private String key;
  private double defaultValue;
  private double lastHasChangeValue = defaultValue;

  /**
   * Create a new TunableNumber
   *
   * @param dashboardKey Key ono dashboard
   */
  public TunableNumber(String dashboardKey) {
    this.key = TABLE_KEY + "/" + dashboardKey;
  }

  /**
   * Create a new TunableNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public TunableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    setDefault(defaultValue);
  }

  /**
   * Get the default value for the number that has been set
   *
   * @return The default value
   */
  public double getDefault() {
    return defaultValue;
  }

  /**
   * Set the default value of the number
   *
   * @param defaultValue The default value
   */
  public void setDefault(double defaultValue) {
    this.defaultValue = defaultValue;
    if (TUNING_MODE) {
      SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode
   *
   * @return The current value
   */
  public double get() {
    return TUNING_MODE ? SmartDashboard.getNumber(key, defaultValue) : defaultValue;
  }

  /**
   * Checks whether the number has changed since our last check
   *
   * @return True if the number has changed since the last time method was called, false otherwise
   */
  public boolean hasChanged() {
    double currentValue = get();
    if (currentValue != lastHasChangeValue) {
      lastHasChangeValue = currentValue;
      return true;
    }

    return false;
  }
}
