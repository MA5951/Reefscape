
package frc.robot.Subsystem.Intake;

import com.ma5951.utils.RobotControl.StatesTypes.StatesConstants;
import com.ma5951.utils.RobotControl.Subsystems.StateControlledSubsystem;

import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystem.Intake.IOs.IntakeIO;
import frc.robot.commands.Swerve.TeleopSwerveController;

public class Intake extends StateControlledSubsystem {
  private static Intake intake;

  private IntakeIO intakeIO = IntakeConstants.getIntakeIO();

  private Intake() {
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

  public boolean IntakeCanMove() {
    return RobotContainer.currentRobotState == RobotConstants.INTAKE && RobotContainer.elevator.atPoint()
        && !getRearSensor() && RobotContainer.arm.atPoint(); 
  }

  public boolean ScoringCanMove() {
    return RobotContainer.currentRobotState == RobotConstants.SCORING && RobotContainer.elevator.atPoint()
        && TeleopSwerveController.atPointForScoring() && RobotContainer.arm.atPoint() && getRearSensor();
  }

  public boolean BallRemovingCanMove() {
    return RobotContainer.currentRobotState == RobotConstants.BALLREMOVING && RobotContainer.elevator.atPoint();
  }

  public boolean SortingCanMove() {
    return RobotContainer.currentRobotState == RobotConstants.SORTING && RobotContainer.arm.atPoint() && RobotContainer.arm.getVelocity() < 1 && RobotContainer.arm.getPosition() > 50 ; 
  }

  @Override
  public boolean canMove() {
    return IntakeCanMove() || ScoringCanMove() || BallRemovingCanMove() || SortingCanMove()
        || getSystemFunctionState() == StatesConstants.MANUEL
        || RobotContainer.intake.getTargetState() == IntakeConstants.HOLD
            && RobotContainer.currentRobotState != RobotConstants.CLIMB;
  }

  public static Intake getInstance() {
    if (intake == null) {
      intake = new Intake();
    }
    return intake;
  }

  @Override
  public void periodic() {
    super.periodic();
    intakeIO.updatePeriodic();
  }
}
