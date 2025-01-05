
package frc.robot.commands.Swerve;

import java.util.function.Supplier;

import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Swerve.SwerveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Subsystem.Swerve.SwerveConstants;

public class RelativAngleAdjustController implements SwerveController {

    private static PIDController pid;
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    private double omega;
    private Supplier<Double> measurment;

    private LoggedDouble omegaLog;
    private LoggedBool atPointLog;

    public RelativAngleAdjustController(double setPointDrgrees, Supplier<Double> getMeasurment) {
        pid = new PIDController(
                SwerveConstants.RELATIV_THATA_KP,
                SwerveConstants.RELATIV_THATA_KI,
                SwerveConstants.RELATIV_THATA_KD);
        measurment = getMeasurment;
        omegaLog = new LoggedDouble("/Subsystems/Swerve/Controllers/Relativ Adjust/Omega Speed");
        atPointLog = new LoggedBool("/Subsystems/Swerve/Controllers/Relativ Adjust/At Point");
        pid.setTolerance(SwerveConstants.RELATIV_ANGLE_PID_TOLORANCE);
        pid.setSetpoint(setPointDrgrees);
    }

    public ChassisSpeeds update() {
        omega = pid.calculate(measurment.get());
        chassisSpeeds.omegaRadiansPerSecond = pid.calculate(measurment.get());

        omegaLog.update(omega);
        atPointLog.update(getAtPoint());

        return chassisSpeeds;
    }

    public boolean getAtPoint() {
        return pid.atSetpoint();
    }

    public void setSetPoint(double setPointDrgrees) {
        pid.setSetpoint(setPointDrgrees);
    }

    public void setMeasurment(Supplier<Double> getMeasurment) {
        measurment = getMeasurment;
    }

    public void updatePID(PIDController pidController) {
        pid = pidController;
    }

}
