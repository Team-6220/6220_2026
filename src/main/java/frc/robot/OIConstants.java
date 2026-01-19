package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;

public final class OIConstants {
  public static final int kDriverControllerPort = 0;

  public static final double kDeadband = 0.085;

  public static final int translationAxis = XboxController.Axis.kLeftY.value;
  public static final int strafeAxis = XboxController.Axis.kLeftX.value;
  public static final int rotationAxis = XboxController.Axis.kRightX.value;

  /*Start from zero after dead band.
   * Eg. if your deadband is .05
   *     if you don't have this function the minimum input would be .05
   *     if you use this function the input would be 0 when the joystick reading is at .05
   */
  public static double modifyMoveAxis(double value) {
    // Deadband
    if (Math.abs(value) < kDeadband) {
      return 0;
    }

    // Change the axis
    double b = .1;
    double a = .5;
    double x = value;
    if (value >= 0) {
      return b + (1 - b) * (a * Math.pow(x, 3) + (1 - a) * x);
    } else {
      return -b + (1 - b) * (a * Math.pow(x, 3) + (1 - a) * x);
    }
    // value = Math.copySign(value * value, value);

    // return value;
  }

  public static double modifyRotAxis(double value) {
    // Deadband
    if (Math.abs(value) < kDeadband) {
      return 0;
    }

    // Change the axis
    double b = .05;
    double a = .2;
    if (value >= 0) {
      return b + (1 - b) * (a * Math.pow(value, 3) + (1 - a) * value);
    } else {
      return -b + (1 - b) * (a * Math.pow(value, 3) + (1 - a) * value);
    }
  }

  public static double modifyVoltageAxis(double value) {
    // Deadband
    if (Math.abs(value) < kDeadband) {
      return 0;
    }

    // Change the axis
    double b = 0.1;
    double a = 0.5;
    double x = value;

    // Calculate modified value
    double modifiedValue;
    if (value >= 0) {
      modifiedValue = b + (1 - b) * (a * Math.pow(x, 3) + (1 - a) * x);
    } else {
      modifiedValue = -b + (1 - b) * (a * Math.pow(x, 3) + (1 - a) * x);
    }

    // Convert to voltage (assuming input ranges from 0 to 1 corresponds to 0 to 12 volts)
    double voltage = modifiedValue * 12;

    return voltage;
  }

  public static double[] getDriverInputs(XboxController driver) {
    double[] inputs = new double[3];

    inputs[0] = OIConstants.modifyMoveAxis(-driver.getRawAxis(translationAxis));
    inputs[1] = OIConstants.modifyMoveAxis(-driver.getRawAxis(strafeAxis));
    inputs[2] = OIConstants.modifyRotAxis(-driver.getRawAxis(rotationAxis));

    inputs[0] = MathUtil.applyDeadband(inputs[0], OIConstants.kDeadband);
    inputs[1] = MathUtil.applyDeadband(inputs[1], OIConstants.kDeadband);
    inputs[2] = MathUtil.applyDeadband(inputs[2], OIConstants.kDeadband);

    int invert = (Constants.isRed.equals("red")) ? -1 : 1;

    inputs[0] *= invert;
    inputs[1] *= invert;

    inputs[0] *= SwerveConstants.maxSpeed;
    inputs[1] *= SwerveConstants.maxSpeed;
    inputs[2] *= SwerveConstants.maxAngularVelocity;
    return inputs;
  }
}
