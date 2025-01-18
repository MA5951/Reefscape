
package frc.robot;

import com.ma5951.utils.RobotControl.DeafultRobotContainer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotControl.Field;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Arm.ArmConstants;
import frc.robot.Subsystem.Elevator.Elevator;
import frc.robot.Subsystem.Elevator.ElevatorConstants;
import frc.robot.Subsystem.Intake.Intake;
import frc.robot.Subsystem.Intake.IntakeConstants;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveAutoFollower;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;
import frc.robot.Subsystem.Vision.Vision;
import frc.robot.Utils.CommandUtil;
import frc.robot.commands.Swerve.TeleopSwerveController;

public class RobotContainer extends DeafultRobotContainer {

  public static SwerveSubsystem swerve;
  public static Vision vision;
  public static PoseEstimator poseEstimator;
  public static SwerveAutoFollower swerveAutoFollower;
  public static Intake intake;
  public static Arm arm;
  public static Elevator elevator;

  public RobotContainer() {
    super(
        PortMap.Controllers.driveID,
        PortMap.Controllers.operatorID,
        PortMap.Controllers.driveRumbleID,
        PortMap.Controllers.operatorRumbleID);
    swerve = SwerveSubsystem.getInstance();
    vision = Vision.getInstance();
    poseEstimator = PoseEstimator.getInstance();
    swerveAutoFollower = SwerveAutoFollower.getInstance();
    intake = Intake.getInstance();
    arm = Arm.getInstance();
    elevator = Elevator.getInstance();

    @SuppressWarnings("unused")
    SuperStructure superStructure = new SuperStructure();

    configureBindings();
    setUpAutoCommands();
  }

  public void setUpAutoCommands() {
    // setAutoOptions(null);
  }

  public void configureTeleopCommands() {
    CommandScheduler.getInstance().setDefaultCommand(swerve,
    new TeleopSwerveController(driverController));
    // CommandScheduler.getInstance().setDefaultCommand(intake,
    // new IntakeDeafultCommand());
    // CommandScheduler.getInstance().setDefaultCommand(arm,
    // new ArmDeafultCommand());

  }

  public static void setIDLE() {
    setCurrentState(RobotConstants.IDLE);
    intake.setTargetState(IntakeConstants.IDLE);
    arm.setTargetState(ArmConstants.IDLE);
    elevator.setTargetState(ElevatorConstants.IDLE);
  }

  public static void setINTAKE() {
    setCurrentState(RobotConstants.INTAKE);
    intake.setTargetState(IntakeConstants.INTAKE);
    arm.setTargetState(ArmConstants.INTAKE);
    elevator.setTargetState(ElevatorConstants.INTAKE);
  }

  public static void setSCORING() {
    setCurrentState(RobotConstants.SCORING);
    intake.setTargetState(IntakeConstants.HOLD);
    arm.setTargetState(ArmConstants.SCORING);
    elevator.setTargetState(ElevatorConstants.SCORING);
  }

  public static void setBALLREMOVING() {
    setCurrentState(RobotConstants.BALLREMOVING);
    intake.setTargetState(IntakeConstants.BALLREMOVING);
    arm.setTargetState(ArmConstants.BALLREMOVING);
    elevator.setTargetState(ElevatorConstants.BALLREMOVING);
  }

  public static void setCLIMB() {
    setCurrentState(RobotConstants.CLIMB);
    intake.setTargetState(IntakeConstants.IDLE);
    arm.setTargetState(ArmConstants.IDLE);
    elevator.setTargetState(ElevatorConstants.CLIMB);
  }

  public static void setSORTING() {
    setCurrentState(RobotConstants.SORTING);
    intake.setTargetState(IntakeConstants.SORTING);
    arm.setTargetState(ArmConstants.IDLE);
    elevator.setTargetState(ElevatorConstants.IDLE);
  }

  private void configureBindings() {

    // Update Offset
    new Trigger(() -> driverController.getTriangleButton())
        .onTrue(Do(() -> TeleopSwerveController.driveController.updateDriveHeading()));

    // Manuel Vision Update
    new Trigger(() -> driverController.getPSButton())
        .onTrue(Do(() -> Vision.getInstance().updateOdometry()));

    // Intake
    new Trigger(() -> driverController.getL1Button()
        || driverController.getR1Button() && !SuperStructure.hasGamePiece() && SuperStructure.isDistanceToIntake())
        .onTrue(Do(() -> setINTAKE()));

    new Trigger(
        () -> currentRobotState == RobotConstants.INTAKE && SuperStructure.hasGamePiece() && intake.getRearSensor())
        .onTrue(Do(() -> setSORTING()));

    // Scoring
    new Trigger(() -> driverController.getL1Button() && SuperStructure.hasGamePiece()
        && currentRobotState != RobotConstants.SORTING)
        .onTrue(Do(() -> SuperStructure.setScoringLocation(Field.ScoringLocation.LEFT)));

    new Trigger(() -> driverController.getR1Button() && SuperStructure.hasGamePiece()
        && currentRobotState != RobotConstants.SORTING)
        .onTrue(Do(() -> SuperStructure.setScoringLocation(Field.ScoringLocation.RIGHT)));

    new Trigger(() -> currentRobotState == RobotConstants.SCORING && !SuperStructure.hasGamePiece()
        && SuperStructure.isDistanceToCloseArm()).onFalse(Do(() -> setIDLE()));

    // Ball Removing
    new Trigger(() -> driverController.getL1Button()
        || driverController.getR1Button() && !SuperStructure.hasGamePiece() && !SuperStructure.isDistanceToIntake())
        .onTrue(Do(() -> setBALLREMOVING()));

    // Climb
    new Trigger(() -> driverController.getSquareButton() && currentRobotState == RobotConstants.IDLE)
        .onTrue(Do(() -> setCLIMB()));

    // IDLE
    new Trigger(() -> driverController.getTouchpadButton()).onTrue(Do(() -> setIDLE()));
  }

  private Command Do(Runnable toRun) {
    return CommandUtil.instantOf(toRun);
  }

}