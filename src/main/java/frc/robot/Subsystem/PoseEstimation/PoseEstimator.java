
package frc.robot.Subsystem.PoseEstimation;


import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Logger.LoggedPose2d;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystem.Swerve.SwerveConstants;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;

public class PoseEstimator {
    private static PoseEstimator poseEstimator;

    private SwerveDrivePoseEstimator robotPoseEstimator;
    private SwerveSubsystem swerve = SwerveSubsystem.getInstance();

    private LoggedPose2d estimatedRobotPose;
    private LoggedDouble lastVisionUpdateLog;
  

    public PoseEstimator() {
        robotPoseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.kinematics , swerve.getRotation2d() , 
        new  SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        },
        new Pose2d(2,2, new Rotation2d()),
        PoseEstimatorConstants.ODOMETRY_DEVS,//Oodmetry Devs
        PoseEstimatorConstants.VISION_DEVS);//Vision Devs

        estimatedRobotPose = new LoggedPose2d("/Pose Estimator/Estimated Robot Pose");
        lastVisionUpdateLog = new LoggedDouble("/Pose Estimator/Last VisionUpdate");
    }

    public void resetPose(Pose2d pose) {
        robotPoseEstimator.resetPosition(swerve.getRotation2d() , swerve.getSwerveModulePositions() ,pose) ;
    }
    
    public void updateOdometry(SwerveModulePosition[] wheelPositions , Rotation2d yaw , double timestemp) {
        robotPoseEstimator.updateWithTime(timestemp,yaw, wheelPositions);
    }

    public void updateVision(Pose2d pose , double timestamp) {
        lastVisionUpdateLog.update(Timer.getFPGATimestamp());
        robotPoseEstimator.addVisionMeasurement(pose, timestamp);
    }

    public Pose2d getEstimatedRobotPose() {
        return robotPoseEstimator.getEstimatedPosition();
    }

    public void update() {
        estimatedRobotPose.update(getEstimatedRobotPose());        
    }

    public static PoseEstimator getInstance() {
        if (poseEstimator == null) {
          poseEstimator = new PoseEstimator();
        }
        return poseEstimator;
        }

}
