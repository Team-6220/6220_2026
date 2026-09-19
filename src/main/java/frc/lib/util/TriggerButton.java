package frc.lib.util;

import org.wpilib.driverstation.XboxController;
import org.wpilib.command2.button.Trigger;

public class TriggerButton extends Trigger {

  public TriggerButton(XboxController controller, XboxController.Axis axis) {
    super(() -> controller.getAxis(axis) >= 0.2);
  }
}
