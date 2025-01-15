
package frc.robot;

import com.ma5951.utils.RobotControl.DeafultRobotContainer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

    configureBindings();
    setUpAutoCommands();
    CommandScheduler.getInstance().setDefaultCommand(SwerveSubsystem.getInstance(),
        new TeleopSwerveController(driverController));
  }


  public void setUpAutoCommands() {
    // setAutoOptions(null);
  }

  public void configureTeleopCommands() {
    // CommandScheduler.getInstance().setDefaultCommand(swerve.getInstance(),
    // new TeleopSwerveController(driverController));
    // CommandScheduler.getInstance().setDefaultCommand(intake,
    // new IntakeDeafultCommand());
    // CommandScheduler.getInstance().setDefaultCommand(arm,
    // new ArmDeafultCommand());

  }

  public void setIDLE() {
    setCurrentState(RobotConstants.IDLE);
    intake.setTargetState(IntakeConstants.IDLE);
    arm.setTargetState(ArmConstants.IDLE);
    elevator.setTargetState(ElevatorConstants.IDLE);
  }

  public void setINTAKE() {
    setCurrentState(RobotConstants.INTAKE);
    intake.setTargetState(IntakeConstants.INTAKE);
    arm.setTargetState(ArmConstants.INTAKE);
    elevator.setTargetState(ElevatorConstants.INTAKE);
  }

  public void setSCORING() {
    setCurrentState(RobotConstants.SCORING);
    intake.setTargetState(IntakeConstants.SCORING);
    arm.setTargetState(ArmConstants.SCORING);
    elevator.setTargetState(ElevatorConstants.SCORING);
  }

  public void setBALLREMOVING() {
    setCurrentState(RobotConstants.BALLREMOVING);
    intake.setTargetState(IntakeConstants.BALLREMOVING);
    arm.setTargetState(ArmConstants.BALLREMOVING);
    elevator.setTargetState(ElevatorConstants.BALLREMOVING);
  }

  public void setCLIMB() {
    setCurrentState(RobotConstants.CLIMB);
    intake.setTargetState(IntakeConstants.IDLE);
    arm.setTargetState(ArmConstants.IDLE);
    elevator.setTargetState(ElevatorConstants.CLIMB);
  }

  public void setSORTING() {
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
        .onTrue(Do(() -> setIDLE()));

  }

  private Command Do(Runnable toRun) {
    return CommandUtil.instantOf(toRun);
  }

}