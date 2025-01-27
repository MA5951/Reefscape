
package frc.robot;

import com.ma5951.utils.RobotControl.DeafultRobotContainer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotControl.Field;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.RobotControl.Field.ScoringLevel;
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
import frc.robot.commands.Arm.ArmDeafultCommand;
import frc.robot.commands.Elevator.ElevatorDeafultCommand;
import frc.robot.commands.Intake.IntakeDeafultCommand;
import frc.robot.commands.Swerve.TeleopSwerveController;

public class RobotContainer extends DeafultRobotContainer {

  public static SwerveSubsystem swerve;
  public static Vision vision;
  public static PoseEstimator poseEstimator;
  public static SwerveAutoFollower swerveAutoFollower;
  public static Intake intake;
  public static Arm arm;
  public static Elevator elevator;
  public static Alliance alliance;
  private static boolean setAllianceData = true;

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

  public static void setUpAutoCommands() {
    // setAutoOptions(null); //TODO add the code
  }

  public static void configureTeleopCommands() {
    CommandScheduler.getInstance().setDefaultCommand(swerve,
        new TeleopSwerveController(driverController));
    CommandScheduler.getInstance().setDefaultCommand(intake,
        new IntakeDeafultCommand());
    CommandScheduler.getInstance().setDefaultCommand(arm,
        new ArmDeafultCommand());
    CommandScheduler.getInstance().setDefaultCommand(elevator,
        new ElevatorDeafultCommand());

  }

  public static void setAllianceData() {
    if (setAllianceData && DriverStation.getAlliance().isPresent()) {
      alliance = DriverStation.getAlliance().get();
      Field.setAllianceReefFaces(alliance);
    }
  }

  public static void setIDLE() {
    setCurrentState(RobotConstants.IDLE);
    intake.setTargetState(IntakeConstants.IDLE);
    arm.setTargetState(ArmConstants.IDLE);
    elevator.setTargetState(ElevatorConstants.IDLE);
  }

  public static void setINTAKE() {
    setCurrentState(RobotConstants.INTAKE);
    SuperStructure.updateAngleAdjustController();
    intake.setTargetState(IntakeConstants.INTAKE);
    arm.setTargetState(ArmConstants.INTAKE);
    elevator.setTargetState(ElevatorConstants.INTAKE);
  }

  public static void setSCORING() {
    SuperStructure.setAbsXY();
    SuperStructure.updateAngleAdjustController();
    setCurrentState(RobotConstants.SCORING);
    intake.setTargetState(IntakeConstants.HOLD);
    arm.setTargetState(ArmConstants.SCORING);
    elevator.setTargetState(ElevatorConstants.SCORING);
  }

  public static void setBALLREMOVING() {
    TeleopSwerveController.ballsTimer.reset();
    TeleopSwerveController.ballsTimer.start();
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
    arm.setTargetState(ArmConstants.HOLD);
    elevator.setTargetState(ElevatorConstants.IDLE);
  }

  private static void configureBindings() {

    // Update Offset
    new Trigger(() -> driverController.getTriangleButton())
        .onTrue(Do(() -> TeleopSwerveController.driveController.updateDriveHeading()));

    // Manuel Vision Update
    new Trigger(() -> driverController.getPSButton())
        .onTrue(Do(() -> Vision.getInstance().updateOdometry()));

    // Intake
    new Trigger(() -> driverController.getR1Button() && !SuperStructure.hasGamePiece())
        .onTrue(Do(() -> setINTAKE()));

    new Trigger(
        () -> currentRobotState == RobotConstants.INTAKE && SuperStructure.hasGamePiece() && SuperStructure.isDistanceToCloseArm()
        && intake.getTargetState() == IntakeConstants.IDLE)
        .onTrue(Do(() -> setSORTING()));
    
    
    

    // // Scoring
    new Trigger(() -> driverController.getL1Button() && SuperStructure.hasGamePiece()
        && currentRobotState != RobotConstants.SORTING)
        .onTrue(Do(() -> SuperStructure.updateScoringFace()))
        .onTrue(Do(() -> SuperStructure.setScoringLocation(Field.ScoringLocation.LEFT)))
        .onTrue(Do(() -> setSCORING()));

    new Trigger(() -> driverController.getR1Button() && SuperStructure.hasGamePiece()
        && currentRobotState != RobotConstants.SORTING)
        .onTrue(Do(() -> SuperStructure.updateScoringFace()))
        .onTrue(Do(() -> SuperStructure.setScoringLocation(Field.ScoringLocation.RIGHT)))
        .onTrue(Do(() -> setSCORING()));

    new Trigger(() -> currentRobotState == RobotConstants.SCORING && !SuperStructure.hasGamePiece()
        && SuperStructure.isDistanceToCloseArm()).onTrue(Do(() -> setIDLE()));

    // Ball Removing
    new Trigger(() -> driverController.getL2Button() && !SuperStructure.hasGamePiece())
        .onTrue(Do(() -> setBALLREMOVING())); //Not too close
    
    

    // // Climb
    // new Trigger(() -> driverController.getSquareButton() && currentRobotState == RobotConstants.IDLE)
    //     .onTrue(Do(() -> setCLIMB()));

    // IDLE
    new Trigger(() -> driverController.getTouchpadButton()).onTrue(Do(() -> setIDLE()));

    //Auto Scoring
    new Trigger(() -> driverController.getRawButton(9)).onTrue(Do(() -> SuperStructure.toggleAutoScoring()));

    //Scoring Levels
    new Trigger(() -> driverController.getPOV() == 180).onTrue(Do(() -> SuperStructure.setScoringPreset(ScoringLevel.L1)));

    new Trigger(() -> driverController.getPOV() == 0).onTrue(Do(() -> SuperStructure.setScoringPreset(ScoringLevel.L4)));

    new Trigger(() -> driverController.getPOV() == 270).onTrue(Do(() -> SuperStructure.setScoringPreset(ScoringLevel.L3)));

    new Trigger(() -> driverController.getPOV() == 90).onTrue(Do(() -> SuperStructure.setScoringPreset(ScoringLevel.L2)));
  }

  private static Command Do(Runnable toRun) {
    return CommandUtil.instantOf(toRun);
  }

}