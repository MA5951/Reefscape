// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Vision.IOs;

import com.ma5951.utils.Vision.Limelights.Limelight3G;
import com.ma5951.utils.Vision.Limelights.LimelightHelpers.PoseEstimate;
import com.ma5951.utils.Vision.Limelights.LimelightHelpers.RawDetection;
import com.ma5951.utils.Vision.Limelights.LimelightHelpers.RawFiducial;

import frc.robot.PortMap;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;

/** Add your docs here. */
public class VisionIOReal implements VisionIO{

    private Limelight3G limelight3G;

    public VisionIOReal() {
        limelight3G = new Limelight3G(PortMap.Vision.FRONT_CAMERA_NAME, () -> SwerveSubsystem.getInstance().getAbsYaw());
    }
    
    public PoseEstimate getEstimatedPose() {
        return limelight3G.getEstimatedPose();
    }

    public boolean isTarget() {
        return limelight3G.isTarget();
    }

    public RawDetection getRawDetection(int detectionIndex) {
        return limelight3G.getRawDetection(detectionIndex);
    }

    public RawDetection getRawDetection() {
        return getRawDetection(0);
    }

    public RawFiducial getRawFiducial(int detectionIndex) {
        return limelight3G.getRawFiducial(detectionIndex);
    }

    public RawFiducial getRawFiducial() {
        return getRawFiducial(0);
    }

    public void filterTags(int[] tagsArry) {
        limelight3G.filterTags(tagsArry);
    }

    public double getTx() {
        return limelight3G.getTx();
    }

    public double getTy() {
        return limelight3G.getTy();
    }

    public double getTa() {
        return limelight3G.getTa();
    }

    public int getTargetCount() {
        return limelight3G.getTargetCount();
    }

    public int getTagID() {
        return limelight3G.getTagID();
    }

    public void update() {
        limelight3G.update();
    }
}
