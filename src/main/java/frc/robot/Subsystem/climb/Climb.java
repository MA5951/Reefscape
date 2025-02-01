
package frc.robot.Subsystem.Climb;

import com.ma5951.utils.RobotControl.Subsystems.StateControlledSubsystem;

import frc.robot.Subsystem.Climb.IOs.ClimbIO;

public class Climb extends StateControlledSubsystem {
  private static Climb climb;

  private ClimbIO climbIO = ClimbConstants.getClimbIO();


  private Climb() {
    super(ClimbConstants.SUBSYSTEM_STATES, "Climb");
  }

  public void setVoltage(double volt) {
    climbIO.setVoltage(volt);
  }

  public Boolean getLimitSensor() {
    return climbIO.getLimitSensor();
  }

  public double getCurrent() {
    return climbIO.getMasterCurrent() ;
  }

  public double getAppliedVolts() {
    return climbIO.getMasterAppliedVolts() ;
  }

  public double getVelocity() {
    return climbIO.getMasterVelocity() ;
  }

  public static Climb getInstance() {
    if (climb == null ) {
      climb = new Climb();
    } 
    return climb;
  }

  @Override
  public void periodic() {
    super.periodic();
    climbIO.updatePeriodic();
  }
}
