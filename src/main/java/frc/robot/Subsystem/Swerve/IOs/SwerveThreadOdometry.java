
package frc.robot.Subsystem.Swerve.IOs;

import java.util.Arrays;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.stream.IntStream;

import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedDouble;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.CollisionDtector;
import frc.robot.Subsystem.Swerve.SkidDetector;
import frc.robot.Subsystem.Swerve.SwerveConstants;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;
import frc.robot.Subsystem.Swerve.Util.GyroData;
import frc.robot.Subsystem.Swerve.Util.OdometryConfig;
import frc.robot.Subsystem.Swerve.Util.SwerveModule;
import frc.robot.Subsystem.Swerve.Util.SwerveModuleData;
import frc.robot.Subsystem.Swerve.Util.SwerveOdometry;
import frc.robot.Utils.ModuleLimits;

public class SwerveThreadOdometry implements SwerveOdometry{

    private OdometryConfig config;

    private LoggedBool skidDetectedLog;
    private LoggedBool collisionDetectedLog;
    private LoggedBool stuckDetectedLog;
    private LoggedDouble lastSkidLog;
    private LoggedDouble lastCollisionLog;
    private LoggedDouble lastStuckLog;

    private SwerveSubsystem swerveSubsystem;
    private SkidDetector skidDetector;
    private CollisionDtector collisionDtector;

    private boolean skidDetected;
    private boolean collisionDetected;
    private boolean stuckDetected;
    private double lastSkid;
    private double lastCollid;
    private double lastStuck;

    private SwerveModulePosition[] wheelPositions = new  SwerveModulePosition[4];
    private SwerveModulePosition[] lastPositions = null;
    private double lastTime;
    public double[] timestamps = new double[] {};
    public static final Lock odometryLock = new ReentrantLock();
    public static final Queue<Double> timestampQueue = new ArrayBlockingQueue<>(20);
    private SwerveModuleData[] modulesData;
    private GyroData gyroData;
    private SwerveModule[] modulesArry;
    private ModuleLimits swerveLimits;
    private double avrageCurrent;
    private Debouncer currentDebouncer;
    private boolean includeMeasurement;

    public SwerveThreadOdometry(OdometryConfig Config) {
        swerveSubsystem = SwerveSubsystem.getInstance();
        config = Config;
        skidDetectedLog = new LoggedBool("/Subsystems/Swerve/Odometry/Skid Detected");
        collisionDetectedLog = new LoggedBool("/Subsystems/Swerve/Odometry/Collision Detected");
        lastSkidLog = new LoggedDouble("/Subsystems/Swerve/Odometry/Last Skid");
        lastCollisionLog = new LoggedDouble("/Subsystems/Swerve/Odometry/Last Collision");
        stuckDetectedLog = new LoggedBool("/Subsystems/Swerve/Odometry/Stuck Detected");
        lastStuckLog = new LoggedDouble("/Subsystems/Swerve/Odometry/Last Stuck");
        currentDebouncer = new Debouncer(0.5, DebounceType.kRising);

        skidDetector = new SkidDetector(SwerveConstants.kinematics, () -> SwerveSubsystem.getInstance().getSwerveModuleStates());
        collisionDtector = new CollisionDtector(() -> SwerveSubsystem.getInstance().getGyroData());


        modulesData = swerveSubsystem.getModulesData();
        gyroData = swerveSubsystem.getGyroData();
        modulesArry = swerveSubsystem.getModulesArry();
        swerveLimits = swerveSubsystem.getCurrentLimits();

    }


    public void updateOdometry() {
        odometryLock.lock();

        timestamps =
            timestampQueue.stream().mapToDouble(Double::valueOf).toArray();
        if (timestamps.length == 0) {
        timestamps = new double[] {Timer.getFPGATimestamp()};
        }
        timestampQueue.clear();

        swerveSubsystem.updateHardwereData();
        modulesData = swerveSubsystem.getModulesData();
        gyroData = swerveSubsystem.getGyroData();
        
        odometryLock.unlock();

        includeMeasurement = true;

        skidDetected = Math.abs(skidDetector.getSkiddingRatio() - 1) < config.skidRatio;
        collisionDetected = collisionDtector.getForceVectorSize() > config.collisionForce;

        skidDetected = false;
        collisionDetected = false;

        avrageCurrent = 0;

        for (int i = 0; i < modulesData.length; i++) {
            avrageCurrent += modulesData[i].getDriveCurrent();
        }

        stuckDetected = currentDebouncer.calculate(avrageCurrent / 4 > config.currentWhenStck);


        skidDetectedLog.update(skidDetected);
        collisionDetectedLog.update(collisionDetected);
        stuckDetectedLog.update(stuckDetected);

        if (skidDetected) {
            lastSkid = Timer.getFPGATimestamp();
            lastSkidLog.update(lastSkid);
        }

        if (collisionDetected) {
            lastCollid = Timer.getFPGATimestamp();
            lastCollisionLog.update(lastCollid);
        }

        if (stuckDetected) {
            lastStuck = Timer.getFPGATimestamp();
            lastStuckLog.update(lastStuck);
        }
        
        if (!(((skidDetected && config.updateInSkid) || !skidDetected) && ((collisionDetected && config.updateInCollision) || !collisionDetected) && ((stuckDetected && config.updateWhenStuck) || !stuckDetected))) {
            includeMeasurement = false;
        }
        
        if (includeMeasurement) {
            int minOdometryUpdates =
            IntStream.of(
                    timestamps.length,
                    Arrays.stream(modulesData)
                        .mapToInt(modulesData -> modulesData.getSteerPositionQueue().length)
                        .min()
                        .orElse(0))
                .min()
                .orElse(0);
        minOdometryUpdates = Math.min(gyroData.getYawPositionQueue().length, minOdometryUpdates);
    
        for (int i = 0; i < minOdometryUpdates; i++) {
        int odometryIndex = i;
        Rotation2d yaw = gyroData.getYawPositionQueue()[i];
        for (int a = 0; a < modulesArry.length ; a++) {
            wheelPositions[a] = modulesArry[a].getModulePositions(modulesData[a])[odometryIndex];
        }
        // Filtering based on delta wheel positions
        if (lastPositions != null) {
            double dt = timestamps[i] - lastTime;
            for (int j = 0; j < modulesArry.length; j++) {
            double velocity =
                (wheelPositions[j].distanceMeters
                        - lastPositions[j].distanceMeters)
                    / dt;
            double omega =
                wheelPositions[j].angle.minus(lastPositions[j].angle).getRadians()
                    / dt;
            // Check if delta is too large
            if (Math.abs(omega) > swerveLimits.maxSteeringVelocity() * 5.0
                || Math.abs(velocity) > swerveLimits.maxDriveVelocity() * 5.0) {
                includeMeasurement = false;
                break;
            }
            }
        }

        if (includeMeasurement) {
            lastPositions = wheelPositions;
            PoseEstimator.getInstance().updateOdometry(wheelPositions, yaw, timestamps[i]);
            lastTime = timestamps[i];
        }
        }
        }
        
    }

    public double getLastSkid() {
        return lastSkid;
    }

    public double getLatCollision() {
        return lastCollid;
    }

}
   