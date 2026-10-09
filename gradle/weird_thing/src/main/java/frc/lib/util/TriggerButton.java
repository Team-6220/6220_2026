package frc.lib.util;

import org.wpilib.command2.button.Trigger;
import org.wpilib.driverstation.XboxController;

public class TriggerButton extends Trigger {

  public TriggerButton(XboxController controller, XboxController.Axis axis) {
    super(() -> controller.getAxis(axis) >= 0.2);
  }
}
