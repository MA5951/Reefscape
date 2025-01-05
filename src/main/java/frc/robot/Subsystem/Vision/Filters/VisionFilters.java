
//Linear Velocity
//Angular Velocity
//In field and not in specefied boxes
//Flickering pose 1. stationery flickring 2. distance from odometry (only auto)
//not deafult pose

//Is Flickering detection 

package frc.robot.Subsystem.Vision.Filters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Subsystem.Vision.VisionIO;

@SuppressWarnings("static-access")
public class VisionFilters {
    private VisionIO visionIO;
    private VisionFiltersConfig config;
    private Supplier<Pose2d> robotPoSupplier;
    private Supplier<ChassisSpeeds> robotSpeedsSupplier;
    private Translation2d robotPose;
    private ChassisSpeeds robotSpeeds;
    private Pose2d deafultPose = new Pose2d();
    private Supplier<Double> robotVelocityVectorSupplier;
    private Pose2d visionPose;
    private double robotVelocity;

    public VisionFilters(VisionIO VisionIO, VisionFiltersConfig configuration, Supplier<Pose2d> robotPose,
            Supplier<ChassisSpeeds> robotSpeeds, Supplier<Double> robotVelocityVector) {
        visionIO = VisionIO;
        config = configuration;
        robotPoSupplier = robotPose;
        robotSpeedsSupplier = robotSpeeds;
        robotVelocityVectorSupplier = robotVelocityVector;
    }

    public void updateFilterConfig(VisionFiltersConfig configuration) {
        config = configuration;
    }

    public boolean isValidForGyroReset() {
        return visionIO.getTargetCount() > 1 && visionIO.getRawFiducial().distToCamera < 2
                && visionIO.getRawFiducial().ambiguity < config.AMBIGUITY_FOR_GYRO_RESET &&
                robotSpeeds.vxMetersPerSecond < config.SPEED_FOR_GYRO_RESET &&
                robotSpeeds.vyMetersPerSecond < config.SPEED_FOR_GYRO_RESET &&
                robotSpeeds.omegaRadiansPerSecond < config.SPEED_FOR_GYRO_RESET;
    }

    public boolean isValidForUpdate(Pose2d visionPose2d) {
        visionPose = visionPose2d;
        return inVelocityFilter() && inField() && notInFieldObstacles() && inOdometryRange()
                && shouldUpdateByRobotState() && notDeafultPose() && isVisionMatchingVelocity();
    }

    private boolean inVelocityFilter() {
        robotSpeeds = robotSpeedsSupplier.get();
        return robotSpeeds.vxMetersPerSecond <= config.robotUpdateSpeed.vxMetersPerSecond &&
                robotSpeeds.vyMetersPerSecond <= config.robotUpdateSpeed.vyMetersPerSecond &&
                robotSpeeds.omegaRadiansPerSecond <= config.robotUpdateSpeed.omegaRadiansPerSecond;
    }

    private boolean inField() {
        return config.fieldRectangle.contains(robotPoSupplier.get().getTranslation());
    }

    private boolean notInFieldObstacles() {
        if (config.fieldObstaclesRectangles != null) {
            robotPose = robotPoSupplier.get().getTranslation();
            for (Rectangle2d obstacles : config.fieldObstaclesRectangles) {
                if (obstacles.contains(robotPose)) {
                    return false;
                }
            }
        }

        return true;
    }

    private boolean inOdometryRange() {
        if ((config.visionToOdometryInTeleop && DriverStation.isTeleop()) || DriverStation.isAutonomous()) {
            robotPose = robotPoSupplier.get().getTranslation();
            return robotPose.getDistance(visionPose.getTranslation()) < config.visionToOdometry;
        }
        return true;
    }

    public boolean isFlickering() {
        return isVisionMatchingVelocity();
    }

    private boolean isVisionMatchingVelocity() {
        robotVelocity = robotVelocityVectorSupplier.get();
        if (robotVelocity < config.maxVelocityForVisionVelocityFilter) {
            return (robotPoSupplier.get().getTranslation().getDistance(
                    visionPose.getTranslation()) <= robotVelocity * 0.02 + config.VISION_VELOCITY_TOLERANCE);
        }

        return true;
    }

    private boolean shouldUpdateByRobotState() {
        if ((config.updateInAuto && DriverStation.isAutonomous()) || !DriverStation.isAutonomous()) {
            return true;
        }

        return false;
    }

    private boolean notDeafultPose() {
        return visionIO.getEstimatedPose().pose != deafultPose;
    }

}
