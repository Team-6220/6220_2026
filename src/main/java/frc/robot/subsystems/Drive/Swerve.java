package frc.robot.subsystems.Drive;

import frc.lib.util.RumbleManager;
import frc.lib.util.TunableNumber;
import frc.robot.AutoConstants;
import frc.robot.Constants;
import frc.robot.SwerveConstants;
// import frc.robot.LimelightCalculations;
// import frc.robot.Localization_V2;
// import frc.robot.LimelightHelpers;
import frc.robot.SwerveModule;
// import frc.robot.Constants.frc.robot.config.RobotConfig.SWERVECONFIG;
// import frc.robot.Constants.VisionConstants;
import frc.robot.VisionConstants;
import frc.robot.config.AutoConfig;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.Drive.GyroIO.GyroIOInputs;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.HashMap;
import java.util.LinkedList;

import org.photonvision.targeting.PhotonPipelineResult;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.*;

// import com.pathplanner.lib.*;

public class Swerve extends SubsystemBase {

  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others.
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  /**
   * Standard deviations of model states. Increase these numbers to trust your model's state
   * estimates less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians,
   * then meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, 0.05);

  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global
   * measurements from vision less. This matrix is in the form [x, y, theta]ᵀ, with units in meters
   * and radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs =
      VecBuilder.fill(1.5, 1.5, Double.MAX_VALUE);

  public SwerveModule[] mSwerveMods;
  private final GyroIO gyro;
  private final GyroIOInputs gyroInputs = new GyroIOInputs();
  private boolean isAutoTurning;

  /**PID controller using rad */
  private ProfiledPIDController turnPidController;
  // private ProfiledPIDController xPidController;
  // private ProfiledPIDController yPidController;
  

  private HashMap<Double, Rotation2d> gyro_headings = new HashMap<Double, Rotation2d>();
  private LinkedList<Double> gyro_timestamps = new LinkedList<Double>();

  public ShuffleboardTab fieldPoseTab = Shuffleboard.getTab("Field Pose 2d tab (map)");

  public Field2d field2d = new Field2d();

  private double lastTurnUpdate;
  private double autoTurnHeading;

  private final int swerveAlignUpdateSecond = 20;

  // private final TunableNumber autoAngularKP = new TunableNumber("Angular kP", AutoConstants.angular_kP);
  // private final TunableNumber autoAngularKI = new TunableNumber("Angular kI", AutoConstants.angular_kI);
  // private final TunableNumber autoAngularKIzone = new TunableNumber("Angular kIzone ", AutoConstants.angular_kIzone);
  // private final TunableNumber autoAngularKD = new TunableNumber("Angular Kd", AutoConstants.angular_kD);
  // private final TunableNumber autoAngularMaxVelDegPerSec =
  //     new TunableNumber("Angular MaxVel Deg per sec", AutoConstants.maxAngularVelocityRadPerSec.in(DegreesPerSecond));
  // private final TunableNumber autoAngularMaxAccelDegPerSecSq =
  //     new TunableNumber("Angular Accel Deg per sec per sec", AutoConstants.maxAngularAcceleratRadPerSecSq.in(DegreesPerSecondPerSecond));
  // private final TunableNumber autoAngularToleranceDeg =
  //     new TunableNumber("Angular Tolerance Deg", AutoConstants.angularTolerance.in(Degrees));

  // private final TunableNumber autoTranslationKP =
  //     new TunableNumber("auto T kP", AutoConstants.translation_kP);
  // private final TunableNumber autoTranslationKI =
  //     new TunableNumber("auto T kI", AutoConstants.translation_kI);
  // private final TunableNumber autoTranslationKD =
  //     new TunableNumber("auto T kD", AutoConstants.translation_kD);

  private boolean autoIsOverShoot = false, isAuto = false;

  // private double targetX;
  // private double targetY;
  // private double targetYaw;
  // private double targetPitch;

  PhotonVisionSubsystem s_Photon = PhotonVisionSubsystem.getInstance(VisionConstants.cameraNames);

  PhotonPipelineResult result;

  private SwerveModulePosition[] positions = {
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()
  };

  private final SwerveDrivePoseEstimator poseEstimator;
  // private final SwerveDriveOdometry odometer;

  RobotConfig config;

  /** initializes the swerve drive and sets up the variables and constants */
  public Swerve() {
    gyro = RobotBase.isSimulation() ? new GyroIOSim() : new GyroIONavX(); // or Pigeon version case SIM -> new GyroIOSim(); };
    // configureAutoBuilder();

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, frc.robot.config.RobotConfig.SWERVECONFIG.backRightMod0()),
          new SwerveModule(1, frc.robot.config.RobotConfig.SWERVECONFIG.backLeftMod1()),
          new SwerveModule(2, frc.robot.config.RobotConfig.SWERVECONFIG.frontRightMod2()),
          new SwerveModule(3, frc.robot.config.RobotConfig.SWERVECONFIG.frontLeftMod3())
        };

    poseEstimator =
        new SwerveDrivePoseEstimator(
            frc.robot.config.RobotConfig.SWERVECONFIG.kinematics(),
            new Rotation2d(),
            positions,
            new Pose2d(),
            stateStdDevs,
            visionMeasurementStdDevs);
    // odometer = new SwerveDriveOdometry(Constants.frc.robot.config.RobotConfig.SWERVECONFIG.kinematics(), new
    // Rotation2d(0), positions);

  turnPidController = new ProfiledPIDController(
      AutoConfig.angularKP.get(),
      AutoConfig.angularKI.get(),
      AutoConfig.angularKD.get(),
      new TrapezoidProfile.Constraints(
          AutoConfig.angularMaxAccelRad(),
          AutoConfig.angularMaxAccelRad()
      )
  );

  turnPidController.setIZone(AutoConfig.angularKIzone.get());
  turnPidController.setTolerance(AutoConfig.angularToleranceRad());

    turnPidController.enableContinuousInput(-(Math.PI/2.0), (Math.PI/2.0));

    // xPidController = new ProfiledPIDController(xKP.get(), xKI.get(), xKD.get(), new
    // TrapezoidProfile.Constraints(xMaxVel.get(), xMaxAccel.get()));
    // xPidController.setIZone(Constants.frc.robot.config.RobotConfig.SWERVECONFIG.xIZone);
    // xPidController.setTolerance(Constants.frc.robot.config.RobotConfig.SWERVECONFIG.xTolerance);
    // xPidController.enableContinuousInput(-180, 180);

    // yPidController = new ProfiledPIDController(yKP.get(), yKI.get(), yKD.get(), new
    // TrapezoidProfile.Constraints(yMaxVel.get(), yMaxAccel.get()));
    // yPidController.setIZone(Constants.frc.robot.config.RobotConfig.SWERVECONFIG.yIZone);
    // yPidController.setTolerance(Constants.frc.robot.config.RobotConfig.SWERVECONFIG.yTolerance);
    // yPidController.enableContinuousInput(-180, 180);

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback(
        (poses) -> field2d.getObject("path").setPoses(poses));
    // Shuffleboard.getTab("Field Pose 2d tab (map)").add("Field 2d", field2d);
    // SmartDashboard.putData("Field", field2d);
    // ModuleConfig swerveModuleConfig = new
    // ModuleConfig(wheelRadius,frc.robot.config.RobotConfig.SWERVECONFIG.maxSpeed,1.0,krackonX60, /);

    // try{
    // config = RobotConfig.fromGUISettings();
    // } catch (Exception e) {
    config =
        new RobotConfig(
            Constants.robotMass,
            Constants.robotMOI,
            SwerveConstants.swerveModuleConfig,
            frc.robot.config.RobotConfig.SWERVECONFIG.kinematics().getModules()); // see
    // https://pathplanner.dev/robot-config.html#bumper-config-options
    // for more details on what you need to set robotconfig up manuelly
    // Also https://pathplanner.dev/api/java/com/pathplanner/lib/config/RobotConfig.html for API
    // e.printStackTrace();
    // }
    createShuffleOutputs();
  }

  /**
   * @param translation the 2d position on where the robot is
   * @param rotation the rotation of the robot
   * @param fieldRelative is the robot driving using the field's directions or the robot's
   *     directions?
   * @param isOpenLoop open loop: takes input directly from controller without feedback from the
   *     output, close loop vice versa
   */
  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        frc.robot.config.RobotConfig.SWERVECONFIG.kinematics().toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getHeading())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, frc.robot.config.RobotConfig.SWERVECONFIG.maxSpeed());

    // set all the modules
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putString(
          "Mod " + mod.moduleNumber + " Swerve Module State",
          swerveModuleStates[mod.moduleNumber].toString());
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /** passes stop driving to all modules */
  public void stopDriving() {
    for (SwerveModule mod : mSwerveMods) {
      mod.stopDriving();
    }
  }

  /** swerve auto init */
  public void configureAutoBuilder() {
    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getRobotRelativeSpeeds,
        (speeds, feedforwards) -> driveRobotRelative(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(
            AutoConfig.translationKP.get(),
            AutoConfig.translationKI.get(),
            AutoConfig.translationKD.get()
            ),
            new PIDConstants(
              AutoConfig.angularKP.get(),
              AutoConfig.angularKI.get(),
              AutoConfig.angularKD.get()
            )),
        config,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
  }

  /**
   * @param robotRelativeSpeeds the speed in m/s
   */
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates =
        frc.robot.config.RobotConfig.SWERVECONFIG.kinematics().toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  /** Resets the odometer value */
  public void resetOdometry(Pose2d pose2d) {
    System.out.println("Reset Odometry: " + pose2d.getX() + ", " + pose2d.getY());
    this.positions[0] = new SwerveModulePosition();
    this.positions[1] = new SwerveModulePosition();
    this.positions[2] = new SwerveModulePosition();
    this.positions[3] = new SwerveModulePosition();
    poseEstimator.resetPosition(getGyroYaw(), positions, pose2d);
  }

  /** Get's the chassis speed of the robot in ROBOT RELATIVE SPEED */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds chassisSpeeds =
        frc.robot.config.RobotConfig.SWERVECONFIG.kinematics().toChassisSpeeds(getModuleStates());
    return chassisSpeeds;
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, frc.robot.config.RobotConfig.SWERVECONFIG.maxSpeed());

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  /**
   * @return list of the states of the modules
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  /**
   * @return positions of the modules
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public Pose2d getPose() {

    return poseEstimator.getEstimatedPosition();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public double getHeadingDegrees() {
    return getPose().getRotation().getDegrees();
  }

  public void setHeading(Rotation2d heading) {
    poseEstimator.resetPosition(
        getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
  }

  public void zeroHeading(XboxController driverController) {
    double offset = Constants.isRed.equals("red") ? 0 : Math.PI;
    poseEstimator.resetPosition(
        getGyroYaw(),
        getModulePositions(),
        new Pose2d(getPose().getTranslation(), new Rotation2d(offset)));
    RumbleManager.rumble(driverController, .2);
  }

  public Rotation2d getGyroYaw() {
    return gyroInputs.yawPosition;
  }

  public boolean isAutoTurning() {
    return isAutoTurning;
  }

  /**
   * @return is it facing toward the target
   */
  public boolean isFacingTurnTarget() {
    return turnPidController.atGoal();
  }

  public void setIsAutoTurning(boolean state) {
    isAutoTurning = state;
  }

  public void setAutoTurnHeading(Angle heading) {
    // autoTurnHeading = heading;
    autoTurnHeading = wrapAngleForTurningPID(heading.in(Degrees));
    resetTurnController();
    turnPidController.setGoal(Radians.convertFrom(autoTurnHeading, Degrees));
  }

  public static double wrapAngleForTurningPID(double angle) {
    angle = angle % 360; // Ensure angle is within 0-360 range
    if (angle > 180) {
      angle -= 360; // Convert angles greater than 180 to negative
    } else if (angle < -180) {
      angle += 360; // Convert angles less than -180 to positive
    }
    return angle;
  }

  public void resetModulesToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  public void resetTurnController() {
    turnPidController.reset(getHeading().getRadians());
    // System.out.println("ResetTurnController");
  }

  public void setTurnControllerGoal(Angle goal) {
    turnPidController.setGoal(goal.in(Radians));
  }

  /**
   * @return gets the angular velocity of turning
   */
  public double getTurnPidSpeed() {

    double speed = turnPidController.calculate(getHeadingDegrees());

    // SmartDashboard.putNumber(" raw speed", speed);

    if (speed > frc.robot.config.RobotConfig.SWERVECONFIG.maxAngularVelocity()) {
      speed = frc.robot.config.RobotConfig.SWERVECONFIG.maxAngularVelocity();
    }
    if (speed < -frc.robot.config.RobotConfig.SWERVECONFIG.maxAngularVelocity()) {
      speed = -frc.robot.config.RobotConfig.SWERVECONFIG.maxAngularVelocity();
    }
    return speed;
  }

  /**
   * @param timestamp the time
   * @return the heading
   */
  public double getHeadingByTimestamp(double timestamp) {
    double timea = 0, timeb = 0;
    if (timestamp > gyro_timestamps.getFirst()) {
      timea = gyro_timestamps.getFirst();
    } else if (timestamp < gyro_timestamps.getLast()) {
      timea = gyro_timestamps.getLast();
    } else {
      for (int i = 0; i < gyro_timestamps.size(); i++) {
        if (gyro_timestamps.get(i) == timestamp) {

        } else if (gyro_timestamps.get(i) < timestamp) {
          timea = gyro_timestamps.get(i - 1);
          timeb = gyro_timestamps.get(i);
          break;
        }
      }
    }
    if (timeb == 0) {
      return gyro_headings.get(timea).getDegrees();
    }
    return ((timestamp - timea)
            / (timeb - timea)
            * (gyro_headings.get(timeb).getDegrees() - gyro_headings.get(timea).getDegrees()))
        + gyro_headings.get(timea).getDegrees();
  }

  public void setIsAuto(boolean isAuto) {
    this.isAuto = isAuto;
    if (!isAuto) {
      autoIsOverShoot = false;
    }
  }

  public boolean getIsAuto() {
    return isAuto;
  }

  public boolean getIsAutoOverShoot() {
    return autoIsOverShoot;
  }

  // public ProfiledPIDController getPidX() {
  //     return xPidController;
  // }

  // public boolean getPidAtGoalX() {
  //     return xPidController.atGoal();
  // }

  // public boolean getPidAtGoalY() {
  //     return yPidController.atGoal();
  // }

  public boolean getPidAtGoalYaw() {
    return turnPidController.atGoal();
  }

  public PhotonPipelineResult getResult() {
    return result;
  }

  // public double getTargetX() {
  //     return targetX;
  // }

  // public double getTargetY() {
  //     return targetY;
  // }

  // public double getTargetYaw() {
  //     return targetYaw;
  // }

  // public double getTargetPitch() {
  //     return targetPitch;
  // }

  @Override
  public void periodic() {
    gyro.updateInputs(gyroInputs);
    SmartDashboard.putBoolean("is Red", Constants.isRed.equals("red"));
    Double timestamp = Timer.getFPGATimestamp();
    // gyro_headings.put(timestamp, getHeading());
    // gyro_timestamps.addFirst(timestamp);
    // if(gyro_timestamps.size() > 60){
    //     timestamp = gyro_timestamps.removeLast();
    //     gyro_headings.remove(timestamp);
    // }
    // for(int i = 0; i < s_Photon.getResults().size(); i++)
    // {
    //     if (s_Photon.getBestTargets().get(i) != null) {
    //         targetX =
    // PhotonVisionCalculations.estimateAdjacent(s_Photon.getBestTargets().get(i).getFiducialId(),
    // i);
    //         targetY =
    // PhotonVisionCalculations.estimateOpposite(s_Photon.getBestTargets().get(i).getFiducialId(),
    // i);
    //     }
    // }
    // targetYaw = result.getBestTarget().yaw;
    // targetPitch = result.getBestTarget().pitch;
    // SmartDashboard.putNumber("distance x", targetX);
    // SmartDashboard.putNumber("distance y", targetY);
    // SmartDashboard.putNumber("hypo",
    // PhotonVisionCalculations.estimateDistance(s_Photon.getBestTargets().get(0).getFiducialId(),
    // 0));
    // SmartDashboard.putNumber("distance yaw", targetYaw);
    // SmartDashboard.putNumber("distance pitch", targetPitch);

    if (timestamp - swerveAlignUpdateSecond >= lastTurnUpdate) {
      lastTurnUpdate = timestamp;
      resetModulesToAbsolute();
      // System.out.println("update!");
    }

    // PhotonVisionSubsystem.updateCamerasPoseEstimation(this, poseEstimator, .00001);
    // LimelightCalculations.updatePoseEstimation(poseEstimator, this);
    // Localization_V2.updateCamerasPoseEstimation(this, poseEstimator,
    // visionMeasurementStdDevConstant.get());
    poseEstimator.update(getGyroYaw(), getModulePositions());

    field2d.setRobotPose(getPose());

    // SmartDashboard.putData("fieldSwerve",field2d);

    if (AutoConfig.angularKP.hasChanged() || AutoConfig.angularKD.hasChanged() || AutoConfig.angularKD.hasChanged()) {
      turnPidController.setPID(AutoConfig.angularKP.get(), AutoConfig.angularKD.get(), AutoConfig.angularKD.get());
      turnPidController.reset(getHeading().getDegrees());
    }

    // if (xKP.hasChanged()
    // || xKI.hasChanged()
    // || xKD.hasChanged()) {
    //     xPidController.setPID(xKP.get(), xKI.get(), xKD.get());
    //     xPidController.reset(getPose().getX());
    // }

    // if (yKP.hasChanged()
    // || yKI.hasChanged()
    // || yKD.hasChanged()) {
    //     yPidController.setPID(yKP.get(), yKI.get(), yKD.get());
    //     yPidController.reset(getPose().getY());
    // }
    if (AutoConfig.angularMaxAccelDeg.hasChanged() || AutoConfig.angularMaxVelDeg.hasChanged()) {
      turnPidController.setConstraints(
          new TrapezoidProfile.Constraints(AutoConfig.angularMaxVelDeg.get(), AutoConfig.angularMaxAccelDeg.get()));
      turnPidController.reset(getHeading().getDegrees());
    }
  }

  private void createShuffleOutputs() {
    String title = "Swerve";
    // Shuffleboard.getTab(title).addString("Robot Pose", () -> getPose().toString());
    Shuffleboard.getTab(title).add(field2d);
    Shuffleboard.getTab(title)
        .addNumber("where the bot think it is swerve X", () -> getPose().getX());
    Shuffleboard.getTab(title)
        .addNumber("where the bot think it is swerve Y", () -> getPose().getY());
    Shuffleboard.getTab(title)
        .addNumber("where the bot think it is swerve degree", () -> getPose().getRotation().getDegrees());
    // SmartDashboard.putString("getRobotPoseField 2d", field2d.getRobotPose().toString());

    for (SwerveModule mod : mSwerveMods) {
      // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder",
      // mod.getCANcoder().getDegrees());
      Shuffleboard.getTab(title)
          .addNumber("Mod " + mod.moduleNumber + " CANcoder", () -> mod.getCANcoder().getDegrees());
      Shuffleboard.getTab(title)
          .addNumber(
              "Mod " + mod.moduleNumber + " Angle", () -> mod.getPosition().angle.getDegrees());
      Shuffleboard.getTab(title)
          .addNumber(
              "Mod " + mod.moduleNumber + " Velocity", () -> mod.getState().speedMetersPerSecond);
      Shuffleboard.getTab(title)
          .addNumber("Mod " + mod.moduleNumber + "setAngle", () -> mod.getDesiredState());
    }
    Shuffleboard.getTab(title).addNumber("Real Heading", () -> getHeading().getDegrees());
    Shuffleboard.getTab(title).addNumber("Auto Turn Heading", () -> autoTurnHeading);
    Shuffleboard.getTab(title)
        .addNumber("Turn Controller Setpoint", () -> turnPidController.getSetpoint().position);
  }
}
