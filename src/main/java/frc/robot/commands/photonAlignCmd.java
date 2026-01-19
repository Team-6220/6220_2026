// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.VisionConstants;
import frc.robot.config.AutoConfig;
import frc.robot.subsystems.Drive.Swerve;
import frc.robot.subsystems.Vision.PhotonVisionSubsystem;
import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class photonAlignCmd extends Command {
  private Swerve s_Swerve;
  private PhotonVisionSubsystem s_Photon;

  // private final TunableNumber xKP = new TunableNumber("x kP", SwerveConstants.xKP);
  // private final TunableNumber xKI = new TunableNumber("x kI", SwerveConstants.xKI);
  // private final TunableNumber xKD = new TunableNumber("x kD", SwerveConstants.xKD);
  // private final TunableNumber xMaxVel = new TunableNumber("x MaxVel", SwerveConstants.xMaxVel);
  // private final TunableNumber xMaxAccel = new TunableNumber("x Accel",
  // SwerveConstants.xMaxAccel);

  // private final TunableNumber yKP = new TunableNumber("y kP", SwerveConstants.yKP);
  // private final TunableNumber yKI = new TunableNumber("y kI", SwerveConstants.yKI);
  // private final TunableNumber yKD = new TunableNumber("y kD", SwerveConstants.yKD);
  // private final TunableNumber yMaxVel = new TunableNumber("y MaxVel", SwerveConstants.yMaxVel);
  // private final TunableNumber yMaxAccel = new TunableNumber("y Accel",
  // SwerveConstants.yMaxAccel);
  private int cameraNum;
  private double xSetpoint, ySetpoint;
  private int lockedFiducialID = -1;
  private PIDController xcontroller =
      new PIDController(
          AutoConfig.translationKP.get(),
          AutoConfig.translationKI.get(),
          AutoConfig.translationKD.get());
  private PIDController ycontroller =
      new PIDController(
          AutoConfig.translationKP.get(),
          AutoConfig.translationKI.get(),
          AutoConfig.translationKD.get());

  // private PhotonTrackedTarget bestTarget;

  /** Creates a new photonAlign. */
  public photonAlignCmd(int cameraNum, Swerve s_Swerve, double xSetpoint, double ySetpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Photon = PhotonVisionSubsystem.getInstance(VisionConstants.cameraNames);
    this.s_Swerve = s_Swerve;
    addRequirements(s_Photon, s_Swerve);
    this.cameraNum = cameraNum;
    this.xSetpoint = xSetpoint;
    this.ySetpoint = ySetpoint;
  }

  // public photonAlignCmd(int cameraNum, Swerve s_Swerve, double offset) {
  //   // Use addRequirements() here to declare subsystem dependencies.
  //   s_Photon = PhotonVisionSubsystem.getInstance(VisionConstants.cameraNames);
  //   this.s_Swerve = s_Swerve;
  //   addRequirements(s_Photon, s_Swerve);
  //   this.cameraNum = cameraNum;
  // }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Swerve.resetTurnController();
    // s_Swerve.setXYGoal(s_Swerve.getTargetX(), s_Swerve.getTargetY());
    System.out.print("Photon vision cmd initilized");
    // offsetX = VisionConstants.aprilTagCoordsX[s_Photon.getBestTarget().get(cameraNum -
    // 1).getFiducialId()] -
    // PhotonVisionCalculations.estimateOpposite(s_Photon.getBestTarget().get(cameraNum).getFiducialId(), cameraNum);
    // offsetY = VisionConstants.aprilTagCoordsY[s_Photon.getBestTarget().get(cameraNum -
    // 1).getFiducialId()] -
    // PhotonVisionCalculations.estimateAdjacent(s_Photon.getBestTarget().get(cameraNum).getFiducialId(), cameraNum);

    // call initPhoton here so that it will declare objects when camera is plugged in while the code
    // is running
    // call initphoton also so that things will get cleared out if something disconnects
    s_Photon.initPhoton();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.print("Photon vision cmd running");
    s_Photon.updatePhoton();
    if (s_Photon.getResults().containsKey(cameraNum)
        && s_Photon.getResults().get(cameraNum) != null
        && !s_Photon.getResults().get(cameraNum).isEmpty()) {
      List<PhotonTrackedTarget> bestTarget = s_Photon.getBestTargets().get(cameraNum);
      SmartDashboard.putNumber("lockedInNum", lockedFiducialID);
      if (bestTarget != null) {
        for (PhotonTrackedTarget tar : bestTarget) {
          if (lockedFiducialID == -1) {
            lockedFiducialID = tar.getFiducialId();
          }

          if (tar.getFiducialId() == lockedFiducialID) {
            Transform3d currentPose = tar.getBestCameraToTarget();

            xcontroller.setSetpoint(xSetpoint);
            ycontroller.setSetpoint(ySetpoint);
            double xout = xcontroller.calculate(currentPose.getX());
            double yout = ycontroller.calculate(currentPose.getY());
            double thetaout = s_Swerve.getTurnPidSpeed();
            SmartDashboard.putNumber("x pid out", xout);
            SmartDashboard.putNumber("y pid out", yout);
            SmartDashboard.putNumber("theta pid out", thetaout);
            s_Swerve.setAutoTurnHeading(Degrees.of(0));
            s_Swerve.drive(new Translation2d(-xout * 2, -yout * 2), -thetaout, false, false);
            SmartDashboard.putNumber("camera to pose x", currentPose.getX());
            SmartDashboard.putNumber("camera to pose y", currentPose.getY());
            SmartDashboard.putNumber("camera to pose z", currentPose.getZ());

            SmartDashboard.putNumber("id", tar.fiducialId);
            SmartDashboard.putNumber("pitch", tar.pitch);
            SmartDashboard.putNumber("yaw", tar.yaw);
            SmartDashboard.putNumber("ambiguity", tar.poseAmbiguity);
            SmartDashboard.putNumber("skew", tar.skew);
          } else {
            s_Swerve.stopDriving();
          }

          // s_Swerve.setAutoTurnHeading(VisionConstants.aprilTagAngle[bestTarget.fiducialId - 1]);
        }
      }
    } else {
      System.err.println(
          "Something's wrong with photon,paste this line and search it globally to find it and look"
              + " at possible errors in the comment");
      /*Potential problem
       * 1. Coprocessor not powered
       * 2. Camera not connected
       * 3. Photonvision not seen on networktable
       */
      end(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("PHOTON ENDED");
    s_Swerve.stopDriving();
    lockedFiducialID = -1;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_Photon.getResults().containsKey(cameraNum)
        && s_Photon.getResults().get(cameraNum) != null
        && s_Photon
            .getResults()
            .get(cameraNum)
            .isEmpty(); // if there's no tag automatically stop it from driving
  }
}
