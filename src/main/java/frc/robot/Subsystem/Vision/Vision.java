// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Vision;

import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Logger.LoggedInt;
import com.ma5951.utils.Logger.LoggedPose2d;
import com.ma5951.utils.Vision.Limelights.LimelightHelpers;
import com.ma5951.utils.Vision.Limelights.LimelightHelpers.PoseEstimate;
import com.ma5951.utils.Vision.Limelights.LimelightHelpers.RawDetection;
import com.ma5951.utils.Vision.Limelights.LimelightHelpers.RawFiducial;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;
import frc.robot.Subsystem.Vision.Filters.VisionFilters;
import frc.robot.Subsystem.Vision.IOs.VisionIO;

public class Vision extends SubsystemBase {
  private static Vision vision;

  private VisionIO visionIO = VisionConstants.getVisionIO();

  private VisionFilters visionFilters = new VisionFilters(visionIO, VisionConstants.AUTO_FILTERS_CONFIG,
      () -> PoseEstimator.getInstance().getEstimatedRobotPose(),
      () -> SwerveSubsystem.getInstance().getRobotRelativeSpeeds(),
      () -> SwerveSubsystem.getInstance().getVelocityVector());

  private LoggedPose2d visionPose2dLog;
  private LoggedDouble tXLog;
  private LoggedDouble tYLog;
  private LoggedBool hasTargetLog;
  private LoggedInt targetCountLog;
  private LoggedBool isValidLog;
  private LoggedBool isValidForResetLog;
  private PoseEstimate visionPoseEstimate;
  private boolean isUpdateForOdometry;
  private boolean isUpdateGyro;
  private boolean didUpdatedGyro = false;

  private LoggedDouble corner0X;
  private LoggedDouble corner0Y;
  private LoggedDouble corner1X;
  private LoggedDouble corner1Y;
  private LoggedDouble corner2X;
  private LoggedDouble corner2Y;
  private LoggedDouble corner3X;
  private LoggedDouble corner3Y;

  private RawDetection rawFiducial;

  public Vision() {
    visionPose2dLog = new LoggedPose2d("/Subsystems/Vision/Vision Pose");
    tXLog = new LoggedDouble("/Subsystems/Vision/Tx");
    tYLog = new LoggedDouble("/Subsystems/Vision/Ty");
    hasTargetLog = new LoggedBool("/Subsystems/Vision/Has Target");
    targetCountLog = new LoggedInt("/Subsystems/Vision/Target Count");
    isValidLog = new LoggedBool("/Subsystems/Vision/Is Valid For Update");
    isValidForResetLog = new LoggedBool("/Subsystems/Vision/Is Valid For Reset");

    corner0X = new LoggedDouble("/Subsystems/Vision/Corners/0 X");
    corner0Y = new LoggedDouble("/Subsystems/Vision/Corners/0 Y");
    corner1X = new LoggedDouble("/Subsystems/Vision/Corners/1 X");
    corner1Y = new LoggedDouble("/Subsystems/Vision/Corners/1 Y");
    corner2X = new LoggedDouble("/Subsystems/Vision/Corners/2 X");
    corner2Y = new LoggedDouble("/Subsystems/Vision/Corners/2 Y");
    corner3X = new LoggedDouble("/Subsystems/Vision/Corners/3 X");
    corner3Y = new LoggedDouble("/Subsystems/Vision/Corners/3 Y");
  }

  public Pose2d getPoseForRelativReefAlign() {
    return new Pose2d( getTa(), getTx(),Rotation2d.kZero);
  }

  public void filterTags(int[] tagsArry) {
    visionIO.filterTags(tagsArry);
  }

  public LimelightHelpers.PoseEstimate getPoseEstimate() {
    return visionIO.getEstimatedPose();
  }

  public double getDirectDistanceToCamera() {
    return visionIO.getRawFiducial().distToCamera;
  }

  public double getTrigoDistanceToCamera() {
    if (getTagID() >= 0 && getTagID() - 1 < 0) {
      return -1;
    }
    double deltaHight = VisionConstants.TAG_HIGHTS[6] - VisionConstants.ROBOT_TO_CAMERA_XYZ.getZ();
    double deltaAngle = getTy() + VisionConstants.ROBOT_TO_CAMERA_ROTATION.getX(); // TODO: Cheack with rader what axis
                                                                                   // should it be
    return deltaHight / Math.tan(Math.toRadians(deltaAngle));
  }

  public double getTx() {
    return visionIO.getTx();
  }

  public double getTy() {
    return visionIO.getTy();
  }

  public double getTa() {
    return visionIO.getTa();
  }

  public boolean isTarget() {
    return visionIO.isTarget();
  }

  public int getTagID() {
    return visionIO.getTagID();
  }

  public int getTargetCount() {
    return visionIO.getTargetCount();
  }

  public void updateOdometry() {
    PoseEstimator.getInstance().updateVision(visionPoseEstimate.pose, visionPoseEstimate.timestampSeconds);
  }

  public static Vision getInstance() {
    if (vision == null) {
      vision = new Vision();
    }
    return vision;
  }

  @Override
  public void periodic() {
    visionIO.update();
    rawFiducial = visionIO.getRawDetection();

    if (DriverStation.isAutonomous()) {
      visionFilters.updateFilterConfig(VisionConstants.AUTO_FILTERS_CONFIG);
    } else {
      visionFilters.updateFilterConfig(VisionConstants.TELEOP_FILTERS_CONFIG);
    }

    visionPoseEstimate = visionIO.getEstimatedPose();

    isUpdateForOdometry = visionFilters.isValidForUpdate(visionPoseEstimate.pose);
    isUpdateGyro = visionFilters.isValidForGyroReset();

    visionPose2dLog.update(visionPoseEstimate.pose);
    tXLog.update(visionIO.getTx());
    tYLog.update(visionIO.getTy());
    hasTargetLog.update(visionIO.isTarget());
    targetCountLog.update(visionIO.getTargetCount());
    isValidLog.update(isUpdateForOdometry);
    isValidForResetLog.update(isUpdateGyro);

    if (isUpdateForOdometry) {
      updateOdometry();
    }

    if (!didUpdatedGyro) {
      if (isUpdateGyro) {
        didUpdatedGyro = true;
        SwerveSubsystem.getInstance().getGyro().updateOffset();
      }
    }

    corner0X.update(rawFiducial.corner0_X);
    corner0Y.update(rawFiducial.corner0_Y);
    corner1X.update(rawFiducial.corner1_X);
    corner1Y.update(rawFiducial.corner1_Y);
    corner2X.update(rawFiducial.corner2_X);
    corner2Y.update(rawFiducial.corner2_Y);
    corner3X.update(rawFiducial.corner3_X);
    corner3Y.update(rawFiducial.corner3_Y);

  }
}
