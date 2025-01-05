package frc.robot.Subsystem.Vision;

import java.util.List;
import java.util.stream.Collectors;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ma5951.utils.Utils.GeomUtil;
import com.ma5951.utils.Vision.Limelights.LimelightHelpers.PoseEstimate;
import com.ma5951.utils.Vision.Limelights.LimelightHelpers.RawDetection;
import com.ma5951.utils.Vision.Limelights.LimelightHelpers.RawFiducial;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystem.Swerve.SwerveConstants;

public class VisionSim implements VisionIO {

    private VisionSystemSim visionSim;
    private AprilTagFieldLayout tagLayout;
    private SimCameraProperties cameraProp;
    private PhotonCamera camera;
    private PhotonCameraSim cameraSim;
    private PhotonPipelineResult result;
    private PhotonPoseEstimator poseEstimator;
    // private List<Integer> lastFilterArry;
    private PoseEstimate toReturn = new PoseEstimate();

    public VisionSim() {
        visionSim = new VisionSystemSim("main");
        try {
            tagLayout = AprilTagFieldLayout
                    .loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

        } catch (Exception e) {
        }

        visionSim.addAprilTags(tagLayout);

        cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(86));
        cameraProp.setFPS(38);
        cameraProp.setCalibError(0.125, 0.04);// 0.6 0.2
        cameraProp.setAvgLatencyMs(30);
        cameraProp.setLatencyStdDevMs(5);

        camera = new PhotonCamera("SimCam");
        cameraSim = new PhotonCameraSim(camera, cameraProp);

        cameraSim.setTargetSortMode(PhotonTargetSortMode.Largest);
        cameraSim.setMaxSightRange(5);

        visionSim.addCamera(cameraSim, VisionConstants.ROBOT_TO_CAMERA);
        visionSim.update(new Pose2d(2, 2, new Rotation2d()));

        poseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                VisionConstants.ROBOT_TO_CAMERA);

        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableDrawWireframe(true);

        // filterTags(new int[] {0});

    }

    @Override
    public PoseEstimate getEstimatedPose() {

        toReturn = new PoseEstimate();
        poseEstimator.update(result).ifPresent((estimator) -> {
            toReturn = new PoseEstimate(estimator.estimatedPose.toPose2d(), Timer.getFPGATimestamp(), 0d, 0, 0d, 0d, 0d,
                    new RawFiducial[] {},true);
        });
        return toReturn;
    }

    @Override
    public boolean isTarget() {
        return result.hasTargets();
    }

    @Override
    public RawDetection getRawDetection(int detectionIndex) {
        return getRawDetection();
    }

    @Override
    public RawDetection getRawDetection() {
        if (isTarget()) {
            return new RawDetection(getTagID(), getTx(), getTy(), getTa(), 0, 0, 0, 0, 0, 0, 0, 0);
        }
        return new RawDetection(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    @Override
    public RawFiducial getRawFiducial(int detectionIndex) {
        return getRawFiducial();
    }

    @Override
    public RawFiducial getRawFiducial() {
        if (isTarget()) {
            return new RawFiducial(getTagID(), getTx(), getTy(), getTa(),
                    result.targets.get(0).bestCameraToTarget.getTranslation().getDistance(new Translation3d()),
                    result.targets.get(0).bestCameraToTarget.getTranslation().getDistance(new Translation3d()),
                    result.targets.get(0).poseAmbiguity);
        }

        return new RawFiducial(0, 0, 0, 0, 0, 0, 0);
    }

    @Override
    public void filterTags(int[] tagsArry) {
        // lastFilterArry = new ArrayList<Integer>(tagsArry.length);
        // for (int i : tagsArry) {
        // lastFilterArry.add(i);
        // }
    }

    @Override
    public double getTx() {
        if (isTarget()) {
            return result.getBestTarget().yaw;
        }
        return 0d;

    }

    @Override
    public double getTy() {

        if (isTarget()) {
            return result.getBestTarget().pitch;
        }
        return 0d;
    }

    @Override
    public double getTa() {
        if (isTarget()) {
            return result.getBestTarget().area;
        }
        return 0d;
    }

    @Override
    public int getTargetCount() {
        if (isTarget()) {
            return result.targets.size();

        }
        return 0;
    }

    @Override
    public int getTagID() {
        if (isTarget()) {
            return result.targets.get(0).getFiducialId();

        }
        return 0;
    }

    // private PhotonPipelineResult filterTagsID() {

    //     for (PhotonTrackedTarget tag : result.targets) {
    //         if (lastFilterArry.contains(Integer.valueOf(tag.fiducialId))) {
    //             result.targets.remove(tag);
    //         }
    //     }

    //     return result;
    // }

    public void update() {
        PhotonPipelineResult latestResult = cameraSim.process(
                0,
                GeomUtil.toPose3d((SwerveConstants.SWERVE_DRIVE_SIMULATION.getSimulatedDriveTrainPose()))
                        .plus(VisionConstants.ROBOT_TO_CAMERA),
                tagLayout.getTags().stream()
                        .map(
                                (a) -> new VisionTargetSim(
                                        a.pose, TargetModel.kAprilTag36h11, a.ID))
                        .collect(Collectors.toList()));
        cameraSim.submitProcessedFrame(latestResult);
        visionSim.update(SwerveConstants.SWERVE_DRIVE_SIMULATION.getSimulatedDriveTrainPose());
        List<PhotonPipelineResult> latest = camera.getAllUnreadResults();
        result = latest.get(0);
        // filterTagsID();
    }

}
