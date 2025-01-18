
package frc.robot.commands.Swerve;

import java.util.function.Supplier;

import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Swerve.SwerveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Subsystem.Swerve.SwerveConstants;

public class AngleAdjustController  implements SwerveController {

  private static PIDController pid;
  private ChassisSpeeds chassisSpeeds =  new ChassisSpeeds();
  private Supplier<Double> measurment;
  private double omega;
  private LoggedDouble omegaLog;
  private LoggedBool atPointLog;
  private LoggedDouble setPointLog;
  private LoggedDouble angleLog;

  /**
   * 
   * @param getMeasurment Must be absolute.
   */
  public AngleAdjustController(Supplier<Double> getMeasurment , double setPointDrgrees) {

    pid = new PIDController(
      SwerveConstants.THATA_KP,
      SwerveConstants.THATA_KI,
      SwerveConstants.THATA_KD
    );


    omegaLog = new LoggedDouble("/Subsystems/Swerve/Controllers/Odometry Adjust/Omega Speed");
    atPointLog = new LoggedBool("/Subsystems/Swerve/Controllers/Odometry Adjust/At Point");
    setPointLog = new LoggedDouble("/Subsystems/Swerve/Controllers/Odometry Adjust/Set Point");
    angleLog = new LoggedDouble("/Subsystems/Swerve/Controllers/Odometry Adjust/Angle");

    measurment = getMeasurment;
    pid.setSetpoint(setPointDrgrees);
    pid.setTolerance(SwerveConstants.ANGLE_PID_TOLORANCE);
    pid.enableContinuousInput(-180, 180);
  }

  public ChassisSpeeds update() {

    omega = pid.calculate(measurment.get());
    chassisSpeeds.omegaRadiansPerSecond = omega;

    atPointLog.update(getAtPoint());
    omegaLog.update(omega);
    setPointLog.update(pid.getSetpoint());
    angleLog.update(measurment.get());

    return chassisSpeeds;
  }

  public boolean getAtPoint() {
    return pid.atSetpoint();
  }

  public double getSetPoint() {
    return pid.getSetpoint();
  }

  public void setSetPoint(double setPointDrgrees) {
    pid.setSetpoint(setPointDrgrees);
  }

  public void updatePID(PIDController pidController) {
    pid = pidController;
  }


}
