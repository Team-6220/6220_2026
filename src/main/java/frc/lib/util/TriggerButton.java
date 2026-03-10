package frc.lib.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerButton extends Trigger {

  public TriggerButton(XboxController controller, XboxController.Axis axis) {
    super(() -> controller.getRawAxis(axis.value) >= 0.2);
  }
}
