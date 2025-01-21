
package frc.robot.commands.Swerve;

import java.util.function.Supplier;

import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Logger.LoggedPose2d;
import com.ma5951.utils.Swerve.SwerveController;
import com.ma5951.utils.Utils.ChassisSpeedsUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Subsystem.Swerve.SwerveConstants;
import frc.robot.Utils.MAProfieldPIDController;

public class AutoAdjustXYController implements SwerveController {

    private Supplier<Pose2d> currentPoseSupplier;
    private MAProfieldPIDController xController = new MAProfieldPIDController(
            SwerveConstants.ABS_X_KP, SwerveConstants.ABS_X_KI, SwerveConstants.ABS_X_KD,
            SwerveConstants.ABS_XY_CONSTRAINTS);
    private MAProfieldPIDController yController = new MAProfieldPIDController(
            SwerveConstants.ABS_Y_KP, SwerveConstants.ABS_X_KI, SwerveConstants.ABS_X_KD,
            SwerveConstants.ABS_XY_CONSTRAINTS);
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    private LoggedDouble xSpeedLog;
    private LoggedDouble ySpeedLog;
    private LoggedBool atPointLog;
    private LoggedPose2d targetPoseLog;
    private LoggedPose2d setPointPoseLog;
    private Pose2d targetPose;
    private boolean isFieldRelativ = true;
    private Supplier<Double> measurment;

    public AutoAdjustXYController(Supplier<Pose2d> robotPoseSupplier, Supplier<Double> robotAngle, Pose2d setPoint) { 
        currentPoseSupplier = robotPoseSupplier;
        measurment = robotAngle;

        xSpeedLog = new LoggedDouble("/Subsystems/Swerve/Controllers/XY Adjust Controller/X Speed");
        ySpeedLog = new LoggedDouble("/Subsystems/Swerve/Controllers/XY Adjust Controller/Y Speed");
        atPointLog = new LoggedBool("/Subsystems/Swerve/Controllers/XY Adjust Controller/At Point");
        targetPoseLog = new LoggedPose2d("/Subsystems/Swerve/Controllers/XY Adjust Controller/Goal Point");
        setPointPoseLog = new LoggedPose2d("/Subsystems/Swerve/Controllers/XY Adjust Controller/Set Point");

        xController.setTolerance(SwerveConstants.ABS_XY_TOLORANCE);
        yController.setTolerance(SwerveConstants.ABS_XY_TOLORANCE);

        updateSetPoint(setPoint);

    }

    public AutoAdjustXYController(Supplier<Pose2d> robotPoseSupplier, Supplier<Double> robotAngle) {
        this(robotPoseSupplier, robotAngle, new Pose2d());
    }

    public void setConstrains(Constraints constraints) {
        xController.setConstraints(constraints);
        yController.setConstraints(constraints);
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
        chassisSpeeds.vxMetersPerSecond = xController.calculate(currentPoseSupplier.get().getX());
        chassisSpeeds.vyMetersPerSecond = yController.calculate(currentPoseSupplier.get().getY());

        xSpeedLog.update(chassisSpeeds.vxMetersPerSecond);
        ySpeedLog.update(chassisSpeeds.vyMetersPerSecond);
        atPointLog.update(atPoint());
        targetPoseLog.update(targetPose);
        setPointPoseLog.update(new Pose2d(xController.getSetpoint().position, yController.getSetpoint().position,
                targetPose.getRotation()));

        if (isFieldRelativ) {
            return ChassisSpeedsUtil.FromFieldToRobot(chassisSpeeds, new Rotation2d(
                    Math.toRadians((measurment.get()))));
        }

        return chassisSpeeds;
    }

    public void updateSetPoint(Pose2d setPoint) {
        xController.setGoal(setPoint.getX(), 0);
        yController.setGoal(setPoint.getY(), 0);

        targetPose = setPoint;
    }

    public Pose2d getSetPoint() {
        return targetPose;
    }

    public boolean atPoint() {
        return xController.atGoal() && yController.atGoal();
    }

    public void updateMeaurment(Supplier<Pose2d> newPose2d) {
        currentPoseSupplier = newPose2d;
    }

}