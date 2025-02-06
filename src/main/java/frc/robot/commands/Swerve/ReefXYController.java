
package frc.robot.commands.Swerve;

import java.util.function.Supplier;

import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Swerve.SwerveController;
import com.ma5951.utils.Utils.ChassisSpeedsUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.RobotControl.Field.ScoringLocation;

public class ReefXYController implements SwerveController{

    private PIDController xPidController;
    private PIDController yPidController;
    private Supplier<Double> gyromMeasurment;
    private Supplier<Double> gyroOffset;
    private ChassisSpeeds speeds = new ChassisSpeeds();
    private LoggedDouble distanceToFace;
    private LoggedDouble ySpeed;
    private LoggedDouble xSpeed;
    private Supplier<Double> measurment;
    private double distance;


    public ReefXYController(Supplier<Double> robotAngle, Supplier<Double> gyroOffset , Supplier<Double> inputSupplier) {
        gyromMeasurment = robotAngle;
        this.gyroOffset = gyroOffset;
        measurment = inputSupplier;

        distanceToFace = new LoggedDouble("/Subsystems/Swerve/Controllers/Reef XY/Distance");
        ySpeed = new LoggedDouble("/Subsystems/Swerve/Controllers/Reef XY/Y Speed");
        xSpeed = new LoggedDouble("/Subsystems/Swerve/Controllers/Reef XY/X Speed");

        xPidController = new PIDController(0.069, 0, 0);
        yPidController = new PIDController(2.4, 0, 0);
        xPidController.setSetpoint(-12.15);
        xPidController.setTolerance(2);
        yPidController.setTolerance(0.1);
        yPidController.setSetpoint(0.435);
        speeds.vxMetersPerSecond = 0;
    }



    public ChassisSpeeds update() {

        distance = SuperStructure.scoringFace.getYDistance(RobotContainer.poseEstimator.getEstimatedRobotPose());
        
        speeds.vyMetersPerSecond = xPidController.calculate(measurment.get());

        // if (SuperStructure.geScoringLocation() == ScoringLocation.RIGHT) {
        // speeds.vxMetersPerSecond = -yPidController.calculate(distance);
        // } else {
        //     speeds.vxMetersPerSecond = yPidController.calculate(distance);
        // }

        speeds.vxMetersPerSecond = 0.07;

        ySpeed.update(speeds.vyMetersPerSecond);
        xSpeed.update(speeds.vxMetersPerSecond);

        distanceToFace.update(distance);

        return speeds;

         
    }

    public void updateSetPoint(double setPoint) {
        xPidController.setSetpoint(setPoint);
    }

    public boolean atPoint() {
        return xPidController.atSetpoint() ;
    }
    
}
