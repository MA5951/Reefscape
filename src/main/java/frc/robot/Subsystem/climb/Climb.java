
package frc.robot.Subsystem.climb;

import com.ma5951.utils.RobotControl.Subsystems.StateControlledSubsystem;

import frc.robot.Subsystem.climb.IOs.ClimbIO;

public class Climb extends StateControlledSubsystem {
  private static Climb climb;

  private ClimbIO climbIO = climbConstants.getClimbIO();


  private Climb() {
    super(climbConstants.SUBSYSTEM_STATES, "Climb");
  }

  public void setVoltage(double volt) {
    climbIO.setVoltage(volt);
  }

  public Boolean getFirstSensor() {
    return climbIO.getFirstSensor();
  }

  public Boolean getSecondSensor() {
    return climbIO.getSecondSensor();
  }

  public double getCurrent() {
    return (climbIO.getLeftCurrent() + climbIO.getRightCurrent()) /2 ;
  }

  public double getAppliedVolts() {
    return (climbIO.getLeftAppliedVolts() + climbIO.getRightAppliedVolts()) /2 ;
  }

  public double getVelocity() {
    return (climbIO.getLeftVelocity() + climbIO.getRightVelocity()) /2 ;
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
