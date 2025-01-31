
package frc.robot.commands.Swerve;

import java.util.function.Supplier;

import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Logger.LoggedPose2d;
import com.ma5951.utils.Swerve.SwerveController;
import com.ma5951.utils.Utils.ChassisSpeedsUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Subsystem.Swerve.SwerveConstants;

public class AutoAdjustXYController implements SwerveController {

    private Supplier<Pose2d> currentPoseSupplier;
    private PIDController xController = new PIDController(
            SwerveConstants.ABS_X_KP, SwerveConstants.ABS_X_KI, SwerveConstants.ABS_X_KD);
    private PIDController yController = new PIDController(
            SwerveConstants.ABS_Y_KP, SwerveConstants.ABS_Y_KI, SwerveConstants.ABS_Y_KD);
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    private LoggedDouble xSpeedLog;
    private LoggedDouble ySpeedLog;
    private LoggedBool atPointLog;
    private LoggedPose2d targetPoseLog;
    private Pose2d targetPose;
    private boolean isFieldRelativ = true;
    private Supplier<Double> gyromMeasurment;
    private Supplier<Double> gyroOffset;

    public AutoAdjustXYController(Supplier<Pose2d> robotPoseSupplier, Supplier<Double> robotAngle, Supplier<Double> gyroOffset ,Pose2d setPoint) { 
        currentPoseSupplier = robotPoseSupplier;
        gyromMeasurment = robotAngle;
        this.gyroOffset = gyroOffset;

        xSpeedLog = new LoggedDouble("/Subsystems/Swerve/Controllers/XY Adjust Controller/X Speed");
        ySpeedLog = new LoggedDouble("/Subsystems/Swerve/Controllers/XY Adjust Controller/Y Speed");
        atPointLog = new LoggedBool("/Subsystems/Swerve/Controllers/XY Adjust Controller/At Point");
        targetPoseLog = new LoggedPose2d("/Subsystems/Swerve/Controllers/XY Adjust Controller/Goal Point");

        xController.setTolerance(SwerveConstants.ABS_XY_TOLORANCE);
        yController.setTolerance(SwerveConstants.ABS_XY_TOLORANCE);

        updateSetPoint(setPoint);
        isFieldRelativ = true;

    }

    public AutoAdjustXYController(Supplier<Pose2d> robotPoseSupplier, Supplier<Double> gyroOffset ,Supplier<Double> robotAngle) {
        this(robotPoseSupplier, robotAngle, gyroOffset ,new Pose2d());
    }

    public void setConstrains(Constraints constraints) {
       
    }

    public void setField(boolean field) {
        isFieldRelativ = field;
    }

    public void setPID(double xKp, double xKi, double xKd, double xTolernace,
            double yKp, double yKi, double yKd, double yTolernace) {
        yController.setPID(yKp, yKi, yKd);
        yController.setTolerance(yTolernace);
        xController.setPID(xKp, xKi, xKd);
        xController.setTolerance(xTolernace);
    }

    public ChassisSpeeds update() {
        chassisSpeeds.vxMetersPerSecond =  xController.calculate(currentPoseSupplier.get().getX());
        chassisSpeeds.vyMetersPerSecond = yController.calculate(currentPoseSupplier.get().getY());

        xSpeedLog.update(chassisSpeeds.vxMetersPerSecond);
        ySpeedLog.update(chassisSpeeds.vyMetersPerSecond);
        atPointLog.update(atPoint());
        targetPoseLog.update(targetPose);

        if (isFieldRelativ) {
            return ChassisSpeedsUtil.FromFieldToRobot(chassisSpeeds, new Rotation2d(
                    Math.toRadians(-(gyromMeasurment.get() - gyroOffset.get()))));
        }

        return chassisSpeeds;
        
    }

    public void updateSetPoint(Pose2d setPoint) { 
        xController.setSetpoint(setPoint.getX());
        yController.setSetpoint(setPoint.getY());

        targetPose = setPoint;
    }

    public Pose2d getSetPoint() {
        return targetPose;
    }

    public boolean atPoint() {
        return xController.atSetpoint() && yController.atSetpoint();
    }

    public void updateMeaurment(Supplier<Pose2d> newPose2d) {
        currentPoseSupplier = newPose2d;
    }

}