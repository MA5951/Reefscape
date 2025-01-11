
package frc.robot.Subsystem.Intake;

import com.ma5951.utils.RobotControl.Subsystems.StateControlledSubsystem;

import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.Field;
import frc.robot.Subsystem.Intake.IOs.IntakeIO;

public class Intake extends StateControlledSubsystem {
  private static Intake intake;

  private IntakeIO intakeIO;

  public Intake() {
    super(IntakeConstants.SUBSYSTEM_STATES, "Intake");
  }

  public boolean getFrontSensor() {
    return intakeIO.getFrontSensor();
  }

  public boolean getRearSensor() {
    return intakeIO.getRearSensor();
  }

  public double getCurrent() {
    return intakeIO.getCurrent();
  }

  public double getPosition() {
    return intakeIO.getPosition();
  }

  public double getVelocity() {
    return intakeIO.getVelocity();
  }

  public double getAppliedVolts() {
    return intakeIO.getAppliedVolts();
  }

  public void setNutralMode(boolean isBrake) {
    intakeIO.setNutralMode(isBrake);
  }

  public void setVoltage(double volt) {
    intakeIO.setVoltage(volt);
  }

  public boolean IntakeCANMOVE() {
    return RobotContainer.currentRobotState == RobotConstants.INTAKE
        && RobotConstants.SUPER_STRUCTURE.getGamePiece() == Field.GamePiece.CORAL
        && !getRearSensor()
        ||
        RobotConstants.SUPER_STRUCTURE.getGamePiece() != Field.GamePiece.BALL;
  }

  @Override
  public boolean canMove() {
    return IntakeCANMOVE();
  }

  public static Intake getInstance() {
    if (intake == null) {
      intake = new Intake();
    }
    return intake;
  }

  @Override
  public void periodic() {
    intakeIO.updatePeriodic();
  }
}
