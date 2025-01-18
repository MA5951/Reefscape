
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
import frc.robot.Subsystem.Swerve.SwerveConstants;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;
import frc.robot.Utils.MAProfieldPIDController;

public class AutoAdjustXYController  {

    private Supplier<Pose2d> currentPoseSupplier;
    private MAProfieldPIDController xController = new MAProfieldPIDController(
        SwerveConstants.X_KP, SwerveConstants.X_KI, SwerveConstants.X_KD, SwerveConstants.XY_CONSTRAINTS);
    private MAProfieldPIDController yController  = new MAProfieldPIDController(
        SwerveConstants.Y_KP, SwerveConstants.Y_KI, SwerveConstants.Y_KD, SwerveConstants.XY_CONSTRAINTS);
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    private LoggedDouble xSpeedLog;
    private LoggedDouble ySpeedLog;
    private LoggedBool atPointLog;
    private LoggedPose2d targetPoseLog;
    private LoggedPose2d setPointPoseLog;
    private Pose2d targetPose;

    public AutoAdjustXYController(Supplier<Pose2d> robotPoseSupplier) {
        currentPoseSupplier = robotPoseSupplier;
        
        xSpeedLog = new LoggedDouble("/Subsystems/Swerve/Controllers/XY Adjust Controller/X Speed");
        ySpeedLog = new LoggedDouble("/Subsystems/Swerve/Controllers/XY Adjust Controller/Y Speed");
        atPointLog = new LoggedBool("/Subsystems/Swerve/Controllers/XY Adjust Controller/At Point");
        targetPoseLog = new LoggedPose2d("/Subsystems/Swerve/Controllers/XY Adjust Controller/Goal Point");
        setPointPoseLog = new LoggedPose2d("/Subsystems/Swerve/Controllers/XY Adjust Controller/Set Point");

        xController.setTolerance(SwerveConstants.XY_TOLORANCE);
        yController.setTolerance(SwerveConstants.XY_TOLORANCE);
        
        

    }

    public void setPID(MAProfieldPIDController xController , MAProfieldPIDController yController) {
        this.xController = xController;
        this.yController = yController;
    }

    public ChassisSpeeds update(boolean isFieldRelativ) {
        chassisSpeeds.vxMetersPerSecond = xController.calculate(currentPoseSupplier.get().getX());
        chassisSpeeds.vyMetersPerSecond = yController.calculate(currentPoseSupplier.get().getY());

        xSpeedLog.update(chassisSpeeds.vxMetersPerSecond);
        ySpeedLog.update(chassisSpeeds.vyMetersPerSecond);
        atPointLog.update(atPoint());
        targetPoseLog.update(targetPose);
        setPointPoseLog.update(new Pose2d(xController.getSetpoint().position, yController.getSetpoint().position, targetPose.getRotation()));

        if (isFieldRelativ) {
        return ChassisSpeedsUtil.FromFieldToRobot(chassisSpeeds, new Rotation2d(
                Math.toRadians((SwerveSubsystem.getInstance().getFusedHeading() - 0))));
        }

        return chassisSpeeds;
    }

    public void updateSetPoint(Pose2d setPoint) {
        xController.setGoal(setPoint.getX() , 0);
        yController.setGoal(setPoint.getY() , 0);

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