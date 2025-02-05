
package frc.robot.Subsystem.Climb;

import com.ma5951.utils.RobotControl.StatesTypes.StatesConstants;
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

  public double getPosition() {
    return climbIO.getPosition();
  }

  public double getCurrent() {
    return climbIO.getMasterCurrent();
  }

  public double getAppliedVolts() {
    return climbIO.getMasterAppliedVolts();
  }

  public double getVelocity() {
    return climbIO.getMasterVelocity();
  }

  public void setServo(double pose) {
    climbIO.setServo(pose);
  }

  public double getServoPose() {
    return climbIO.getServoPose();
  }

  public boolean atAlignAngle() {
    return Math.abs(getPosition() - ClimbConstants.ALIGN_ANGLE) <= ClimbConstants.TOLERANCE;
  }

  private boolean physicalCanMove() {
    return true;//((getPosition() >= ClimbConstants.MIN_ANGLE || getAppliedVolts() < -0.1));
  }

  private boolean alignCanMove() {
    return getTargetState() == ClimbConstants.ALIGN && getPosition() < ClimbConstants.ALIGN_ANGLE;
  }

  private boolean climbCanMove() {
    return getTargetState() == ClimbConstants.CLIMB && getPosition() > ClimbConstants.CLIMB_ANGLE;
  }

  @Override
  public boolean canMove() {
    return alignCanMove() || climbCanMove() || getSystemFunctionState() == StatesConstants.MANUEL; //|| climbCanMove() || getSystemFunctionState() == StatesConstants.MANUEL
  }

  public static Climb getInstance() {
    if (climb == null) {
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
