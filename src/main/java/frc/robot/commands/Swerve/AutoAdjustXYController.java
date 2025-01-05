
package frc.robot.commands.Swerve;

import java.util.function.Supplier;

import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Logger.LoggedPose2d;
import com.ma5951.utils.Swerve.SwerveController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Subsystem.Swerve.SwerveConstants;

public class AutoAdjustXYController implements SwerveController {

    private Supplier<Pose2d> currentPoseSupplier;
    private ProfiledPIDController xController = new ProfiledPIDController(
        SwerveConstants.XY_KP, SwerveConstants.XY_KI, SwerveConstants.XY_KD, SwerveConstants.XY_CONSTRAINTS);
    private ProfiledPIDController yController  = new ProfiledPIDController(
        SwerveConstants.XY_KP, SwerveConstants.XY_KI, SwerveConstants.XY_KD, SwerveConstants.XY_CONSTRAINTS);
    private ChassisSpeeds chassisSpeeds;
    private LoggedDouble xSpeedLog;
    private LoggedDouble ySpeedLog;
    private LoggedBool atPointLog;
    private LoggedPose2d targetPoseLog;
    private Pose2d targetPose;

    public AutoAdjustXYController(Supplier<Pose2d> robotPoseSupplier) {
        currentPoseSupplier = robotPoseSupplier;
        
        xSpeedLog = new LoggedDouble("/Subsystems/Swerve/Controllers/XY Adjust Controller/X Speed");
        ySpeedLog = new LoggedDouble("/Subsystems/Swerve/Controllers/XY Adjust Controller/Y Speed");
        atPointLog = new LoggedBool("/Subsystems/Swerve/Controllers/XY Adjust Controller/At Point");
        targetPoseLog = new LoggedPose2d("/Subsystems/Swerve/Controllers/XY Adjust Controller/Set Point");

        xController.setTolerance(SwerveConstants.XY_TOLORANCE);
        yController.setTolerance(SwerveConstants.XY_TOLORANCE);

    }

    public ChassisSpeeds update() {
        chassisSpeeds.vxMetersPerSecond = xController.calculate(currentPoseSupplier.get().getX());
        chassisSpeeds.vyMetersPerSecond = yController.calculate(currentPoseSupplier.get().getY());

        xSpeedLog.update(chassisSpeeds.vxMetersPerSecond);
        ySpeedLog.update(chassisSpeeds.vyMetersPerSecond);
        atPointLog.update(atPoint());
        targetPoseLog.update(targetPose);

        return chassisSpeeds;
    }

    public void updateSetPoint(Pose2d setPoint) {
        xController.setGoal(setPoint.getX());
        yController.setGoal(setPoint.getY());

        targetPose = setPoint;
    }

    public Pose2d getSetPoint() {
        return targetPose;
    }

    public boolean atPoint() {
        return xController.atGoal() && yController.atGoal();
    }

    


}
