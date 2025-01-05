
package frc.robot.Subsystem.Vision;

import com.ma5951.utils.Vision.Limelights.LimelightHelpers;

public interface VisionIO {


    LimelightHelpers.PoseEstimate getEstimatedPose();

    boolean isTarget();

    LimelightHelpers.RawDetection getRawDetection(int detectionIndex);

    LimelightHelpers.RawDetection getRawDetection();

    LimelightHelpers.RawFiducial getRawFiducial(int detectionIndex) ;

    LimelightHelpers.RawFiducial getRawFiducial();

    void filterTags(int[] tagsArry);

    double getTx();

    double getTy();

    double getTa();

    int getTargetCount();

    int getTagID();

    void update();

}
