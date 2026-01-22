package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.RumbleManager;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.VisionConstants;
import frc.robot.config.AutoConfig;
import frc.robot.subsystems.Drive.GyroIO.GyroIOInputs;
import frc.robot.subsystems.Vision.PhotonVisionSubsystem;
import java.util.HashMap;
import java.util.LinkedList;
import org.photonvision.targeting.PhotonPipelineResult;

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

  public SwerveModule[] mSwerveMods = new SwerveModule[4]; // BR, BL, FR, FL
  private final GyroIO gyro;
  private final GyroIOInputs gyroInputs = new GyroIOInputs();
  private boolean isAutoTurning;

  /** PID controller using rad */
  private ProfiledPIDController turnPidController;

  private HashMap<Double, Rotation2d> gyro_headings = new HashMap<Double, Rotation2d>();
  private LinkedList<Double> gyro_timestamps = new LinkedList<Double>();

  public ShuffleboardTab fieldPoseTab = Shuffleboard.getTab("Field Pose 2d tab (map)");

  public Field2d field2d = new Field2d();

  private double lastTurnUpdate;
  private double autoTurnHeading;

  private final int swerveAlignUpdateSecond = 20;

  private boolean autoIsOverShoot = false, isAuto = false;

  PhotonVisionSubsystem s_Photon = PhotonVisionSubsystem.getInstance(VisionConstants.cameraNames);

  PhotonPipelineResult result;

  private SwerveModulePosition[] positions = {
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()
  };

  private final SwerveDrivePoseEstimator poseEstimator;

  RobotConfig config;

  /** initializes the swerve drive and sets up the variables and constants */
  public Swerve() {
    gyro = RobotBase.isSimulation() ? new GyroIOSim() : new GyroIONavX();

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(
              0,
              frc.robot.config.RobotConfig.SWERVECONFIG.backRightMod0(),
              RobotBase.isSimulation()
                  ? new SwerveModuleIOSim()
                  : new SwerveModuleIOTalonFXSparkMax(
                      frc.robot.config.RobotConfig.SWERVECONFIG.backRightMod0())),
          new SwerveModule(
              1,
              frc.robot.config.RobotConfig.SWERVECONFIG.backLeftMod1(),
              RobotBase.isSimulation()
                  ? new SwerveModuleIOSim()
                  : new SwerveModuleIOTalonFXSparkMax(
                      frc.robot.config.RobotConfig.SWERVECONFIG.backLeftMod1())),
          new SwerveModule(
              2,
              frc.robot.config.RobotConfig.SWERVECONFIG.frontRightMod2(),
              RobotBase.isSimulation()
                  ? new SwerveModuleIOSim()
                  : new SwerveModuleIOTalonFXSparkMax(
                      frc.robot.config.RobotConfig.SWERVECONFIG.frontRightMod2())),
          new SwerveModule(
              3,
              frc.robot.config.RobotConfig.SWERVECONFIG.frontLeftMod3(),
              RobotBase.isSimulation()
                  ? new SwerveModuleIOSim()
                  : new SwerveModuleIOTalonFXSparkMax(
                      frc.robot.config.RobotConfig.SWERVECONFIG.frontLeftMod3()))
        };

    poseEstimator =
        new SwerveDrivePoseEstimator(
            frc.robot.config.RobotConfig.SWERVECONFIG.kinematics(),
            new Rotation2d(),
            positions,
            new Pose2d(),
            stateStdDevs,
            visionMeasurementStdDevs);

    turnPidController =
        new ProfiledPIDController(
            AutoConfig.angularKP.get(),
            AutoConfig.angularKI.get(),
            AutoConfig.angularKD.get(),
            new TrapezoidProfile.Constraints(
                AutoConfig.angularMaxAccelRad(), AutoConfig.angularMaxAccelRad()));

    turnPidController.setIZone(AutoConfig.angularKIzone.get());
    turnPidController.setTolerance(AutoConfig.angularToleranceRad());

    turnPidController.enableContinuousInput(-(Math.PI / 2.0), (Math.PI / 2.0));

    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback(
        (poses) -> field2d.getObject("path").setPoses(poses));

    // try{
    // config = RobotConfig.fromGUISettings();
    // } catch (Exception e) {
    config =
        new RobotConfig(
            Constants.robotMass,
            Constants.robotMOI,
            frc.robot.config.RobotConfig.SWERVECONFIG.swerveModuleConfig(),
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
        frc.robot.config.RobotConfig.SWERVECONFIG
            .kinematics()
            .toSwerveModuleStates(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(), translation.getY(), rotation, getHeading())
                    : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, frc.robot.config.RobotConfig.SWERVECONFIG.maxSpeed());

    // set all the modules
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putString(
          "Mod " + mod.getModuleNumber() + " Swerve Module State",
          swerveModuleStates[mod.getModuleNumber()].toString());
      mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
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
                AutoConfig.translationKD.get()),
            new PIDConstants(
                AutoConfig.angularKP.get(),
                AutoConfig.angularKI.get(),
                AutoConfig.angularKD.get())),
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
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, frc.robot.config.RobotConfig.SWERVECONFIG.maxSpeed());

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.getModuleNumber()], false);
    }
  }

  /**
   * @return list of the states of the modules
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.getModuleNumber()] = mod.getState();
    }
    return states;
  }

  /**
   * @return positions of the modules
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.getModuleNumber()] = mod.getPosition();
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
  }

  public void setTurnControllerGoal(Angle goal) {
    turnPidController.setGoal(goal.in(Radians));
  }

  /**
   * @return gets the angular velocity of turning
   */
  public double getTurnPidSpeed() {

    double speed = turnPidController.calculate(getHeadingDegrees());

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

  public boolean getPidAtGoalYaw() {
    return turnPidController.atGoal();
  }

  public PhotonPipelineResult getResult() {
    return result;
  }

  @Override
  public void periodic() {
    gyro.updateInputs(gyroInputs);
    SmartDashboard.putBoolean("is Red", Constants.isRed.equals("red"));
    Double timestamp = Timer.getFPGATimestamp();

    if (timestamp - swerveAlignUpdateSecond >= lastTurnUpdate) {
      lastTurnUpdate = timestamp;
      resetModulesToAbsolute();
      // System.out.println("update!");
    }

    poseEstimator.update(getGyroYaw(), getModulePositions());

    field2d.setRobotPose(getPose());

    if (AutoConfig.angularKP.hasChanged()
        || AutoConfig.angularKD.hasChanged()
        || AutoConfig.angularKD.hasChanged()) {
      turnPidController.setPID(
          AutoConfig.angularKP.get(), AutoConfig.angularKD.get(), AutoConfig.angularKD.get());
      turnPidController.reset(getHeading().getDegrees());
    }

    if (AutoConfig.angularMaxAccelDeg.hasChanged() || AutoConfig.angularMaxVelDeg.hasChanged()) {
      turnPidController.setConstraints(
          new TrapezoidProfile.Constraints(
              AutoConfig.angularMaxVelDeg.get(), AutoConfig.angularMaxAccelDeg.get()));
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
        .addNumber(
            "where the bot think it is swerve degree", () -> getPose().getRotation().getDegrees());

    for (SwerveModule mod : mSwerveMods) {
      Shuffleboard.getTab(title)
          .addNumber(
              "Mod " + mod.getModuleNumber() + " CANcoder", () -> mod.getCANcoder().getDegrees());
      Shuffleboard.getTab(title)
          .addNumber(
              "Mod " + mod.getModuleNumber() + " Angle",
              () -> mod.getPosition().angle.getDegrees());
      Shuffleboard.getTab(title)
          .addNumber(
              "Mod " + mod.getModuleNumber() + " Velocity",
              () -> mod.getState().speedMetersPerSecond);
    }
    Shuffleboard.getTab(title).addNumber("Real Heading", () -> getHeading().getDegrees());
    Shuffleboard.getTab(title).addNumber("Auto Turn Heading", () -> autoTurnHeading);
    Shuffleboard.getTab(title)
        .addNumber("Turn Controller Setpoint", () -> turnPidController.getSetpoint().position);
  }
}
