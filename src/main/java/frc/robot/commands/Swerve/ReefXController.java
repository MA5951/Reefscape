
package frc.robot.commands.Swerve;

import java.util.function.Supplier;

import com.ma5951.utils.Swerve.SwerveController;
import com.ma5951.utils.Utils.ChassisSpeedsUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;

public class ReefXController implements SwerveController{

    private PIDController xPidController;
    private Supplier<Double> gyromMeasurment;
    private Supplier<Double> gyroOffset;
    private ChassisSpeeds speeds = new ChassisSpeeds();
    private Supplier<Double> measurment;


    public ReefXController(Supplier<Double> robotAngle, Supplier<Double> gyroOffset , Supplier<Double> inputSupplier) {
        gyromMeasurment = robotAngle;
        this.gyroOffset = gyroOffset;
        measurment = inputSupplier;
        
        xPidController = new PIDController(0.059, 0, 0);
        xPidController.setSetpoint(-12.15);
        xPidController.setTolerance(1.2);
        speeds.vxMetersPerSecond = 0;
    }



    public ChassisSpeeds update() {
        
        speeds.vyMetersPerSecond = xPidController.calculate(measurment.get());
        System.out.println(speeds.vyMetersPerSecond);
        return speeds;

         
    }

    public void updateSetPoint(double setPoint) {
        xPidController.setSetpoint(setPoint);
    }

    public boolean atPoint() {
        return xPidController.atSetpoint();
    }
    
}
