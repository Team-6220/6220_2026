package frc.robot.subsystems;

import frc.lib.util.TunableNumber;
import frc.robot.Constants;
// import frc.robot.LimelightCalculations;
// import frc.robot.Localization_V2;
//import frc.robot.LimelightHelpers;
import frc.robot.SwerveModule;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
//import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import static frc.robot.Constants.isRed;

import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;

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
import edu.wpi.first.units.MassUnit;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.MutMass;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
     * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
     * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
     */
    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, 0.05);
    
    /**
     * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(1.5, 1.5, Double.MAX_VALUE);


    public SwerveModule[] mSwerveMods;
    public AHRS gyro;
    private boolean isAutoTurning;
    private ProfiledPIDController turnPidController;// ProfiledPIDController creates a "trapazoid" when it speeds up to avoid pulling too much voltage from the battery at once.

    private HashMap<Double, Rotation2d> gyro_headings = new HashMap<Double, Rotation2d>();
    private LinkedList<Double> gyro_timestamps = new LinkedList<Double>();

    public ShuffleboardTab fieldPoseTab = Shuffleboard.getTab("Field Pose 2d tab (map)");

    public Field2d field2d = new Field2d();

    private double lastTurnUpdate;
    private double autoTurnHeading;

    private final TunableNumber turnKP = new TunableNumber("turn kP", Constants.SwerveConstants.turnKP);
    private final TunableNumber turnKI = new TunableNumber("turn kI", Constants.SwerveConstants.turnKI);
    private final TunableNumber turnKD = new TunableNumber("turn Kd", Constants.SwerveConstants.turnKD);
    private final TunableNumber turnMaxVel = new TunableNumber("turn MaxVel", Constants.SwerveConstants.turnMaxVel);
    private final TunableNumber turnMaxAccel = new TunableNumber("turn Accel", Constants.SwerveConstants.turnMaxAccel);

    private final TunableNumber autoRkP = new TunableNumber("auto R kP", Constants.SwerveConstants.rotation_kP);
    private final TunableNumber autoRkI = new TunableNumber("auto R kI", Constants.SwerveConstants.rotation_kI);
    private final TunableNumber autoRkD = new TunableNumber("auto R kD", Constants.SwerveConstants.rotation_kD);

    private final TunableNumber autoTkP = new TunableNumber("auto T kP", Constants.SwerveConstants.translation_kP);
    private final TunableNumber autoTkI = new TunableNumber("auto T kI", Constants.SwerveConstants.translation_kI);
    private final TunableNumber autoTkD = new TunableNumber("auto T kD", Constants.SwerveConstants.translation_kD);


    private boolean autoIsOverShoot = false, isAuto = false;

    
    public final TunableNumber visionMeasurementStdDevConstant = new TunableNumber("visionStdDev Constant", VisionConstants.visionStdDev);

    private SwerveModulePosition[] positions = {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };
    
    private final SwerveDrivePoseEstimator poseEstimator;
    //private final SwerveDriveOdometry odometer;

    /**
     * initializes the swerve drive and sets up the variables and constants
     */
    public Swerve() {
        gyro = new AHRS(NavXComType.kMXP_SPI, NavXUpdateRate.k200Hz);


        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, SwerveConstants.Mod0.constants),
            new SwerveModule(1, SwerveConstants.Mod1.constants),
            new SwerveModule(2, SwerveConstants.Mod2.constants),
            new SwerveModule(3, SwerveConstants.Mod3.constants)
        };


        poseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.swerveKinematics, new Rotation2d(), positions, new Pose2d(), stateStdDevs, visionMeasurementStdDevs);
        //odometer = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, new Rotation2d(0), positions);

        

        turnPidController = new ProfiledPIDController(turnKP.get(), turnKI.get(), turnKD.get(), new TrapezoidProfile.Constraints(turnMaxVel.get(), turnMaxAccel.get()));
        turnPidController.setIZone(Constants.SwerveConstants.turnIZone);
        turnPidController.setTolerance(Constants.SwerveConstants.turnTolerance);
        turnPidController.enableContinuousInput(-180, 180);

        // Set up custom logging to add the current path to a field 2d widget
        PathPlannerLogging.setLogActivePathCallback((poses) -> field2d.getObject("path").setPoses(poses));
        // Shuffleboard.getTab("Field Pose 2d tab (map)").add("Field 2d", field2d);
        // SmartDashboard.putData("Field", field2d);

         RobotConfig config;
        try{
        config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
        }
        createShuffleOutputs();
    }

    /**
     * @param translation
     * the 2d position on where the robot is
     * @param rotation
     * the rotation of the robot
     * @param fieldRelative
     * is the robot driving using the field's directions or the robot's directions?
     * @param isOpenLoop
     * open loop: takes input directly from controller without feedback from the output, close loop vice versa
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            SwerveConstants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

        //set all the modules
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putString("Mod " + mod.moduleNumber +" Swerve Module State", swerveModuleStates[mod.moduleNumber].toString());
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
            }

    /**
     * passes stop driving to all modules
     */
    public void stopDriving()
    {
        for(SwerveModule mod : mSwerveMods)
        {
            mod.stopDriving();
        }
    }
    /**
     * swerve auto init
     */
    public void configureAutoBuilder() {
        // AutoBuilder.configureHolonomic(
        // this::getPose, // Robot pose supplier
        // this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
        // this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        // this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        // new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
        // new PIDConstants(autoRkP.get(), autoRkI.get(), autoRkD.get()), // Translation PID constants
        // new PIDConstants(autoTkP.get(), autoTkI.get(), autoTkD.get()), // Rotation PID constants
        //         4.5, // Max module speed, in m/s
        //         0.4, // Drive base radius in meters. Distance from robot center to furthest module.
        //         new ReplanningConfig() // Default path replanning config. See the API for the options here
        // ),
        // () -> {
        //     // Boolean supplier that controls when the path will be mirrored for the red alliance
        //     // This will flip the path being followed to the red side of the field.
        //     // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        //     var alliance = DriverStation.getAlliance();
        //     if (alliance.isPresent()) {
        //     return alliance.get() == DriverStation.Alliance.Red;
        //     }
        //     return false;
        // },
        // this // Reference to this subsystem to set requirements
        // );

    //     AutoBuilder.configure(
    //         this::getPose, // Robot pose supplier
    //         this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
    //         this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //         (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
    //         new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
    //                 new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    //                 new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
    //         ),
    //         new RobotConfig(null, null, null, null) , // The robot configuration
    //         () -> {
    //           // Boolean supplier that controls when the path will be mirrored for the red alliance
    //           // This will flip the path being followed to the red side of the field.
    //           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //           var alliance = DriverStation.getAlliance();
    //           if (alliance.isPresent()) {
    //             return alliance.get() == DriverStation.Alliance.Red;
    //           }
    //           return false;
    //         },
    //         this // Reference to this subsystem to set requirements
    // );
    }
    

    /**
     * @param robotRelativeSpeeds
     * the speed in m/s
     */
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    
        SwerveModuleState[] targetStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
      }
    /**
     * Resets the odometer value
     */
    public void resetOdometry(Pose2d pose2d){
        System.out.println("Reset Odometry: " + pose2d.getX() + ", " + pose2d.getY());
        this.positions[0] = new SwerveModulePosition();
        this.positions[1] = new SwerveModulePosition();
        this.positions[2] = new SwerveModulePosition();
        this.positions[3] = new SwerveModulePosition();
        poseEstimator.resetPosition(getGyroYaw(), positions, pose2d);
  }

/**
 * Get's the chassis speed of the robot in ROBOT RELATIVE SPEED
 */

  public ChassisSpeeds getRobotRelativeSpeeds()
  {
    ChassisSpeeds chassisSpeeds = SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
    return chassisSpeeds;

  }

  /**
     * AKA get x value to amp as of map in pathplanner, forward/positvie is away from DRIVER'S POINT OF VIEW at BLUE SIDE
     * @return the forward/backward/x distance from robot to amp
     */
    public double getAmpX()
    {
        //Pose2d currPose = getPose();
        Pose2d ampPose = Constants.isRed.equals("red") ? VisionConstants.AMP_POSE2D_RED : VisionConstants.AMP_POSE2D_BLUE;
        double xDistance = ampPose.getX(); // - currPose.getX() // I don't think we need this
        SmartDashboard.putNumber("forward backward", xDistance);
        return xDistance;
    }

    /**
     * AKA get y value to amp as of map in pathplanner
     * @return the left/right/y distance from robot to amp, right/negative is to the right side of the driver FROM THE BLUE SIDE
     */
    public double getAmpY()
    {
        Pose2d currPose = getPose();
        Pose2d ampPose = Constants.isRed.equals("red") ? VisionConstants.AMP_POSE2D_RED : VisionConstants.AMP_POSE2D_BLUE;
        double yDistance = ampPose.getY() ; //- currPose.getY() // I don't think we need it
        SmartDashboard.putNumber("left and right", yDistance);
        return yDistance;
    }

    /**
     * use for auto
     * @return The amp position based on team color
     */
    public Pose2d getAmpPose()
    {
        return Constants.isRed.equals("red") ? VisionConstants.AMP_POSE2D_RED : VisionConstants.AMP_POSE2D_BLUE;
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    /**
     * @return list of the states of the modules
     */
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    /**
     * @return positions of the modules
     */
    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
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

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    /**
     * @return the direction to the speaker
     */
    public double getHeadingToSpeaker(){
        
        Pose2d currPose = getPose();
        Pose2d speakerPose = Constants.isRed.equals("red") ? VisionConstants.SPEAKER_POSE2D_RED : VisionConstants.SPEAKER_POSE2D_BLUE;
        double angle = Math.toDegrees(Math.atan2(speakerPose.getY() - currPose.getY(), speakerPose.getX() - currPose.getX()));
        angle += (Constants.isRed.equals("red") ? 0 : -180);
        return angle;
    }

    public double getHeadingDegrees()
    {
        return getPose().getRotation().getDegrees();
    }

    public void setHeading(Rotation2d heading){
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        double offset = Constants.isRed.equals("red") ? 0 : Math.PI;
        poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d(offset)));
    }

    public Rotation2d getGyroYaw() {
        return gyro.getRotation2d();
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

    public void setAutoTurnHeading(double heading) {
        autoTurnHeading = heading;
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void resetTurnController() {
        turnPidController.reset(getHeading().getDegrees());
        //System.out.println("ResetTurnController");
    }

    public void setTurnControllerGoal(double goal) {
        turnPidController.setGoal(goal);
    }

    /**
     * @return gets the angular velocity of turning
     */
    public double getTurnPidSpeed() {

        turnPidController.setGoal(autoTurnHeading);

        if (Timer.getFPGATimestamp() - 0.2 > lastTurnUpdate) {
            resetTurnController();
        }
        lastTurnUpdate = Timer.getFPGATimestamp();

    
        double speed = turnPidController.calculate(getHeadingDegrees());

        //SmartDashboard.putNumber(" raw speed", speed);

        if(speed > SwerveConstants.maxAngularVelocity) {
            speed = SwerveConstants.maxAngularVelocity;
        } if (speed < -SwerveConstants.maxAngularVelocity) {
            speed = -SwerveConstants.maxAngularVelocity;
        }
        return speed;
    }


    /**
     * @param timestamp
     * the time
     * @return the heading
     */
    public double getHeadingByTimestamp(double timestamp){
        double timea = 0, timeb = 0;
        if(timestamp > gyro_timestamps.getFirst()){
            timea = gyro_timestamps.getFirst();
        }
        else if(timestamp < gyro_timestamps.getLast()){
            timea = gyro_timestamps.getLast();
        }
        else{
            for(int i = 0; i < gyro_timestamps.size(); i++){
                if(gyro_timestamps.get(i) == timestamp){

                }
                else if(gyro_timestamps.get(i) < timestamp){
                    timea = gyro_timestamps.get(i-1);
                    timeb = gyro_timestamps.get(i);
                    break;
                }
            }
        }
        if (timeb == 0){
            return gyro_headings.get(timea).getDegrees();
        }
        return ((timestamp - timea)/(timeb - timea) * (gyro_headings.get(timeb).getDegrees() - gyro_headings.get(timea).getDegrees())) + gyro_headings.get(timea).getDegrees();
        
    }

    public void setIsAuto(boolean isAuto)
    {
        this.isAuto = isAuto;
        if(!isAuto)
        {
            autoIsOverShoot = false;
        }
    }

    public boolean getIsAuto(){
        return isAuto;
    }

    public boolean getIsAutoOverShoot()
    {
        return autoIsOverShoot;
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("is Red", isRed.equals("red"));
        Double timestamp = Timer.getFPGATimestamp();
        // gyro_headings.put(timestamp, getHeading());
        // gyro_timestamps.addFirst(timestamp);
        // if(gyro_timestamps.size() > 60){
        //     timestamp = gyro_timestamps.removeLast();
        //     gyro_headings.remove(timestamp);
        // }

        if(timestamp - SwerveConstants.swerveAlignUpdateSecond >= lastTurnUpdate)
        {
            lastTurnUpdate = timestamp;
            resetModulesToAbsolute();
            // System.out.println("update!");
        }

        
        // LimelightCalculations.updatePoseEstimation(poseEstimator, this);
        // Localization_V2.updateCamerasPoseEstimation(this, poseEstimator, visionMeasurementStdDevConstant.get());
        poseEstimator.update(getGyroYaw(), getModulePositions());
        
        field2d.setRobotPose(getPose());

        // SmartDashboard.putData("fieldSwerve",field2d);
        

        if (isAuto && ((Constants.isRed.equals("red") && field2d.getRobotPose().getX() < AutoConstants.maxXDistance) || (!Constants.isRed.equals("red") && field2d.getRobotPose().getX() > AutoConstants.maxXDistance)))
        {
            autoIsOverShoot = true;
        }
        else
        {
            autoIsOverShoot = false;
        }

        if(turnKP.hasChanged()
        || turnKD.hasChanged()
        || turnKI.hasChanged()) {
            turnPidController.setPID(turnKP.get(), turnKI.get(), turnKD.get());
            turnPidController.reset(getHeading().getDegrees());
        }
        if(turnMaxAccel.hasChanged() || turnMaxVel.hasChanged()) {
            turnPidController.setConstraints(new TrapezoidProfile.Constraints(turnMaxVel.get(), turnMaxAccel.get()));
            turnPidController.reset(getHeading().getDegrees());
        }
        
    }

    private void createShuffleOutputs() {
        String title = "Swerve";
        // Shuffleboard.getTab(title).addString("Robot Pose", () -> getPose().toString());
        Shuffleboard.getTab(title).add(field2d);
        //SmartDashboard.putString("getRobotPoseField 2d", field2d.getRobotPose().toString());
 
        for(SwerveModule mod : mSwerveMods){
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            Shuffleboard.getTab(title).addNumber("Mod " + mod.moduleNumber + " CANcoder", () -> mod.getCANcoder().getDegrees());
            Shuffleboard.getTab(title).addNumber("Mod " + mod.moduleNumber + " Angle", ()-> mod.getPosition().angle.getDegrees());
            Shuffleboard.getTab(title).addNumber("Mod " + mod.moduleNumber + " Velocity", ()-> mod.getState().speedMetersPerSecond);  
            Shuffleboard.getTab(title).addNumber("Mod " + mod.moduleNumber + "setAngle",()-> mod.getDesiredState());
        }
        Shuffleboard.getTab(title).addNumber("Real Heading", ()-> getHeading().getDegrees());
        Shuffleboard.getTab(title).addNumber("Auto Turn Heading", ()->autoTurnHeading);
        Shuffleboard.getTab(title).addNumber("Turn Controller Setpoint", ()->turnPidController.getSetpoint().position);

        
    }

    /**
     * @deprecated?
     * Gets the closest not position during autonomous
     */
    public int getClosestNotePosition()
    {
        Pose2d rightPose = poseEstimator.getEstimatedPosition().nearest(Arrays.asList(AutoConstants.CENTERNOTE_POSE2DS));
        for(int i = 0; i < AutoConstants.CENTERNOTE_POSE2DS.length; i++)
        {
            if(AutoConstants.CENTERNOTE_POSE2DS[i].equals(rightPose))
            return i;
        }
        return -1;        
    }
}