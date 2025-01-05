
package frc.robot.commands.Swerve;

import java.util.function.Supplier;

import com.ma5951.utils.Swerve.SwerveController;
import com.ma5951.utils.Utils.ChassisSpeedsUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PS5Controller;
import frc.robot.Subsystem.Swerve.SwerveConstants;

public class FieldCentricDriveController implements SwerveController {

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    private PS5Controller controller;
    private Supplier<Boolean> reductionBoolean;
    private Supplier<Double> angleSupplier;
    private double reductionPrecent = 1;
    private double xSpeed;
    private double ySpeed;
    private double turningSpeed;
    private double gyroOffset = 0d;

    public FieldCentricDriveController(PS5Controller Controller, Supplier<Boolean> ReductionSupplier,
            double ReductionPrecent,
            Supplier<Double> AngleSupplier) {
        controller = Controller;
        reductionBoolean = ReductionSupplier;
        reductionPrecent = ReductionPrecent;
        angleSupplier = AngleSupplier;
    }

    public ChassisSpeeds update() {

        xSpeed = -controller.getLeftX();
        ySpeed = controller.getLeftY();
        turningSpeed = controller.getRightX();

        xSpeed = Math.abs(xSpeed) < 0.1 ? 0 : -xSpeed * SwerveConstants.DRIVER_XY_SCALER * SwerveConstants.MAX_VELOCITY;
        ySpeed = Math.abs(ySpeed) < 0.1 ? 0 : -ySpeed * SwerveConstants.DRIVER_XY_SCALER * SwerveConstants.MAX_VELOCITY;
        turningSpeed = Math.abs(turningSpeed) < 0.1 ? 0
                : -turningSpeed * SwerveConstants.DRIVER_THATA_SCALER
                        * SwerveConstants.MAX_ANGULAR_VELOCITY;

        if (reductionBoolean.get()) {
            xSpeed = xSpeed * reductionPrecent;
            ySpeed = ySpeed * reductionPrecent;
            turningSpeed = turningSpeed * reductionPrecent;
        }

        chassisSpeeds.vxMetersPerSecond = xSpeed;
        chassisSpeeds.vyMetersPerSecond = ySpeed;
        chassisSpeeds.omegaRadiansPerSecond = turningSpeed;

        return ChassisSpeedsUtil.FromFieldToRobot(chassisSpeeds, new Rotation2d(
                Math.toRadians((angleSupplier.get() - gyroOffset))));

    }

    public void updateDriveHeading() {
        gyroOffset = angleSupplier.get();
    }

}
