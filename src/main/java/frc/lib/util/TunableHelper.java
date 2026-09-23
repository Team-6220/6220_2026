package frc.lib.util;

import static frc.robot.Constants.*;

import org.wpilib.tunable.TunableBase;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;

public final class TunableHelper {
  private TunableHelper() {}

  /**
   * Same as {@link Tunables#addDouble(String, double)}, but only published to the dashboard when
   * TUNING_MODE is enabled. Otherwise the value is local-only and stays at its initial value.
   *
   * @param key Key on dashboard
   * @param initialValue Initial value
   * @return The tunable double
   */
  public static TunableDouble addDouble(String key, double initialValue) {
    if (TUNING_MODE) {
      return Tunables.addDouble(key, initialValue);
    }
    return TunableDouble.create(initialValue);
  }

  /**
   * Checks whether any of the given tunables have changed, and clears their changed flags.
   * Tunable change flags are sticky until {@link TunableBase#resetChanged()} is called.
   *
   * @param tunables Tunables to check
   * @return True if any tunable changed since the last check, false otherwise
   */
  public static boolean consumeChanged(TunableBase... tunables) {
    boolean changed = false;
    for (TunableBase tunable : tunables) {
      if (tunable.hasChanged()) {
        changed = true;
        tunable.resetChanged();
      }
    }
    return changed;
  }
}
