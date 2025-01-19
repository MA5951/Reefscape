
package frc.robot.Subsystem.Intake;

import com.ma5951.utils.RobotControl.StatesTypes.StatesConstants;
import com.ma5951.utils.RobotControl.Subsystems.StateControlledSubsystem;

import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.Arm.ArmConstants;
import frc.robot.Subsystem.Intake.IOs.IntakeIO;
import frc.robot.commands.Swerve.TeleopSwerveController;

public class Intake extends StateControlledSubsystem {
  private static Intake intake;

  private IntakeIO intakeIO = IntakeConstants.getIntakeIO();

  public Intake() { //TODO change to priveate
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
        && TeleopSwerveController.atPointForScoring() && RobotContainer.arm.atPoint(); //TODO have a game piece 
  }

  public boolean BallRemovingCanMove() {
    return RobotContainer.currentRobotState == RobotConstants.BALLREMOVING && RobotContainer.elevator.atPoint(); // TODO why? 
  }

  public boolean SortingCanMove() {
    return RobotContainer.currentRobotState == RobotConstants.SORTING
        && ((getRearSensor() && intakeIO.getIntendedVoltage() > 0) || !getRearSensor()); //TODO need to be checkt in sim
  }

  @Override
  public boolean canMove() {
    return IntakeCanMove() || ScoringCanMove() || BallRemovingCanMove() || SortingCanMove()
        || getSystemFunctionState() == StatesConstants.MANUEL
        || RobotContainer.arm.getTargetState() == ArmConstants.HOLD //TODO change to inake state 
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
    //TODO call super()
    intakeIO.updatePeriodic();
  }
}
