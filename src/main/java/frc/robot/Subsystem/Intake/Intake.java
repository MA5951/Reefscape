
package frc.robot.Subsystem.Intake;

import com.ma5951.utils.RobotControl.StatesTypes.StatesConstants;
import com.ma5951.utils.RobotControl.Subsystems.StateControlledSubsystem;
import com.ma5951.utils.Utils.BooleanLatch;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.RobotControl.Field.ScoringLevel;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Intake.IOs.IntakeIO;
import frc.robot.commands.Swerve.TeleopSwerveController;

public class Intake extends StateControlledSubsystem {
  private static Intake intake;

  private IntakeIO intakeIO = IntakeConstants.getIntakeIO();
  public static BooleanLatch scoringAtPointLatch;
  public static Debouncer scoringAtPointDebouncer;

  private Intake() {
    super(IntakeConstants.SUBSYSTEM_STATES, "Intake");
    scoringAtPointLatch = new BooleanLatch(() -> RobotContainer.arm.atPoint() );
    scoringAtPointDebouncer = new Debouncer(0.55);
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
    return RobotContainer.currentRobotState == RobotConstants.INTAKE && RobotContainer.elevator.atPoint();
  }

  public boolean ScoringCanMove() {
    return SuperStructure.isScoringAutomatic == true
        ? RobotContainer.currentRobotState == RobotConstants.SCORING && ((RobotContainer.elevator.atPoint()
            && (TeleopSwerveController.atPointForScoring() || (SuperStructure.getScoringPreset() == ScoringLevel.L1 && (RobotContainer.driverController.getL1Button() || RobotContainer.driverController.getR1Button()))) && scoringAtPointLatch.get() && getFrontSensor())
            || getTargetState() == IntakeConstants.HOLD)
        : RobotContainer.currentRobotState == RobotConstants.SCORING && ((RobotContainer.elevator.atPoint()
            && RobotContainer.arm.atPoint() && (getFrontSensor() || getRearSensor()))
            || getTargetState() == IntakeConstants.HOLD) &&
            (RobotContainer.driverController.getL1Button() || RobotContainer.driverController.getR1Button());
  }

  public boolean BallRemovingCanMove() {
    return RobotContainer.currentRobotState == RobotConstants.BALLREMOVING && RobotContainer.elevator.atPoint()
         && RobotContainer.arm.getPosition() > IntakeConstants.BALLREMOVING_ANGLE && !TeleopSwerveController.ballsLatch.get();
  }

  public boolean SortingCanMove() {
    return RobotContainer.currentRobotState == RobotConstants.SORTING && //RobotContainer.arm.atPoint() &&
         RobotContainer.arm.getPosition() > IntakeConstants.SORTING_ANGLE;
  }

  @Override
  public boolean canMove() {
    return RobotContainer.currentRobotState == RobotConstants.HOLDBALL ||  IntakeCanMove() || scoringAtPointDebouncer.calculate(ScoringCanMove()) || BallRemovingCanMove()
    || getTargetState() == IntakeConstants.EJECT
        || SortingCanMove()
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
