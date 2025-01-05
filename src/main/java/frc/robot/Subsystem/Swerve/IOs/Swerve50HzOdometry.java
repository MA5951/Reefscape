
package frc.robot.Subsystem.Swerve.IOs;

import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedDouble;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.CollisionDtector;
import frc.robot.Subsystem.Swerve.SkidDetector;
import frc.robot.Subsystem.Swerve.SwerveConstants;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;
import frc.robot.Subsystem.Swerve.Util.OdometryConfig;
import frc.robot.Subsystem.Swerve.Util.SwerveOdometry;

public class Swerve50HzOdometry implements SwerveOdometry{

    private OdometryConfig odometryConfig;
    private SwerveSubsystem swerveSubsystem;
    private SkidDetector skidDetector;
    private CollisionDtector collisionDtector;

    private LoggedBool skidDetectedLog;
    private LoggedBool collisionDetectedLog;
    private LoggedDouble lastSkidLog;
    private LoggedDouble lastCollisionLog;


    private boolean skidDetected;
    private boolean collisionDetected;
    private double lastSkid;
    private double lastCollid;

    public Swerve50HzOdometry(OdometryConfig Config) {
        swerveSubsystem = SwerveSubsystem.getInstance();
        odometryConfig = Config;
        skidDetectedLog = new LoggedBool("/Subsystems/Swerve/Odometry/Skid Detected");
        collisionDetectedLog = new LoggedBool("/Subsystems/Swerve/Odometry/Collision Detected");
        lastSkidLog = new LoggedDouble("/Subsystems/Swerve/Odometry/Last Skid");
        lastCollisionLog = new LoggedDouble("/Subsystems/Swerve/Odometry/Last Collision");
        skidDetector = new SkidDetector(SwerveConstants.kinematics, () -> SwerveSubsystem.getInstance().getSwerveModuleStates());
        collisionDtector = new CollisionDtector(() -> SwerveSubsystem.getInstance().getGyroData());
    }


    public void updateOdometry() {
        swerveSubsystem.updateHardwereData();

        skidDetected = Math.abs(skidDetector.getSkiddingRatio() - 1) < odometryConfig.skidRatio;
        collisionDetected = collisionDtector.getForceVectorSize() > odometryConfig.collisionForce;

        skidDetectedLog.update(skidDetected);
        collisionDetectedLog.update(collisionDetected);

        if (skidDetected) {
            lastSkid = Timer.getFPGATimestamp();
            lastSkidLog.update(lastSkid);
        }

        if (collisionDetected) {
            lastCollid = Timer.getFPGATimestamp();
            lastCollisionLog.update(lastCollid);
        }
        
        if (((skidDetected && odometryConfig.updateInSkid) || !skidDetected) && ((collisionDetected && odometryConfig.updateInCollision) || !collisionDetected)) {
            PoseEstimator.getInstance().updateOdometry(swerveSubsystem.getSwerveModulePositions(), swerveSubsystem.getRotation2d() , Timer.getFPGATimestamp());
        }
    }

    public double getLastSkid() {
        return lastSkid;
    }

    public double getLatCollision() {
        return lastCollid;
    }


}
   