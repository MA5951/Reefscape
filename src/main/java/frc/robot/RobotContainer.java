
package frc.robot;

import com.ma5951.utils.RobotControl.DeafultRobotContainer;
import com.ma5951.utils.RobotControl.StatesTypes.StatesConstants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotControl.Field;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.RobotControl.Field.ScoringLevel;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Arm.ArmConstants;
import frc.robot.Subsystem.Climb.Climb;
import frc.robot.Subsystem.Climb.ClimbConstants;
import frc.robot.Subsystem.Elevator.Elevator;
import frc.robot.Subsystem.Elevator.ElevatorConstants;
import frc.robot.Subsystem.Intake.Intake;
import frc.robot.Subsystem.Intake.IntakeConstants;
import frc.robot.Subsystem.Leds.Leds;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveAutoFollower;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;
import frc.robot.Subsystem.Vision.Vision;
import frc.robot.Utils.CommandUtil;
import frc.robot.commands.Arm.ArmDeafultCommand;
import frc.robot.commands.Climb.ClimbDeafultCommand;
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
  public static boolean setAllianceData = true;
  public static Climb climb;

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
    climb = Climb.getInstance();

    @SuppressWarnings("unused")
    SuperStructure superStructure = new SuperStructure();
    Field field = new Field();
    Leds.getInstance();

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
    CommandScheduler.getInstance().setDefaultCommand(climb,
    new ClimbDeafultCommand());

  }

  public static void setAllianceData() {
    if (setAllianceData && DriverStation.getAlliance().isPresent()) {
      alliance = DriverStation.getAlliance().get();
      //Field.setAllianceReefFaces(alliance);
      setAllianceData = false;
    }
  }

  public static void setIDLE() {
    setCurrentState(RobotConstants.IDLE);
    intake.setTargetState(IntakeConstants.IDLE);
    arm.setTargetState(ArmConstants.IDLE);
    elevator.setTargetState(ElevatorConstants.IDLE);
    climb.setTargetState(ClimbConstants.IDLE);
  }

  public static void setINTAKE() {
    setCurrentState(RobotConstants.INTAKE);
    SuperStructure.updateAngleAdjustController();
    intake.setTargetState(IntakeConstants.INTAKE);
    arm.setTargetState(ArmConstants.INTAKE);
    elevator.setTargetState(ElevatorConstants.INTAKE);
  }

  public static void setSCORING() {
    Intake.scoringAtPointLatch.reset();
    intake.setTargetState(IntakeConstants.HOLD);
    arm.setTargetState(ArmConstants.SCORING);
    elevator.setTargetState(ElevatorConstants.SCORING);
  }

  public static void setSCORINGALIGN() {
    setCurrentState(RobotConstants.SCORING);
    SuperStructure.isFine = false;
    SuperStructure.isFinalLeft = false;
    SuperStructure.isFinalRight = false;
    SuperStructure.updateScoringFace();
    SuperStructure.setAbsXY();
    SuperStructure.updateAngleAdjustController();
  }

  public static void setBALLREMOVING() {
    SuperStructure.isFine = false;
    arm.ballsPoseLatch.reset();
    SuperStructure.updateScoringFace();
    SuperStructure.setAbsXYBalls();
    TeleopSwerveController.ballsLatch.reset();
    setCurrentState(RobotConstants.BALLREMOVING);
    SuperStructure.updateAngleAdjustController();
    intake.setTargetState(IntakeConstants.BALLREMOVING);
    arm.setTargetState(ArmConstants.BALLREMOVING);
    elevator.setTargetState(ElevatorConstants.BALLREMOVING);
  }

  public static void setCLIMB() {
    setCurrentState(RobotConstants.CLIMB);
    intake.setTargetState(IntakeConstants.IDLE);
    arm.setTargetState(ArmConstants.IDLE);
    elevator.setTargetState(ElevatorConstants.CLIMB);
    climb.setTargetState(ClimbConstants.ALIGN);
  }

  public static void setSORTING() {
    setCurrentState(RobotConstants.SORTING);
    intake.setTargetState(IntakeConstants.SORTING);
    arm.setTargetState(ArmConstants.HOLD);
    elevator.setTargetState(ElevatorConstants.IDLE);
  }

  public static void setSKYHOOK() {
    setCurrentState(RobotConstants.SKYHOOK);
    arm.setTargetState(ArmConstants.SKYHOOK);
    elevator.setTargetState(ElevatorConstants.SKYHOOK);
  }

  public static void setHOLDBALL() {
    setCurrentState(RobotConstants.HOLDBALL);
    arm.setTargetState(ArmConstants.HOLD);
    elevator.setTargetState(ElevatorConstants.IDLE);
    intake.setTargetState(IntakeConstants.SKYHOOK);
  }

  public static void setEject() {
    setCurrentState(RobotConstants.EJECT);
    intake.setTargetState(IntakeConstants.EJECT);
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
        () -> currentRobotState == RobotConstants.INTAKE && SuperStructure.hasGamePiece()
            && SuperStructure.isDistanceToCloseArm()
            && intake.getTargetState() == IntakeConstants.IDLE)
        .onTrue(Do(() -> setSORTING()));

    // // Scoring
    new Trigger(() -> driverController.getL1Button()
        && currentRobotState != RobotConstants.SORTING && SuperStructure.hasGamePiece())
        .onTrue(Do(() -> SuperStructure.setScoringLocation(Field.ScoringLocation.LEFT)))
        .onTrue(Do(() -> setSCORINGALIGN()));

    new Trigger(() -> driverController.getR1Button()
        && currentRobotState != RobotConstants.SORTING && SuperStructure.hasGamePiece())
        .onTrue(Do(() -> SuperStructure.updateScoringFace()))
        .onTrue(Do(() -> SuperStructure.setScoringLocation(Field.ScoringLocation.RIGHT)))
        .onTrue(Do(() -> setSCORINGALIGN()));

    new Trigger(() -> currentRobotState == RobotConstants.SCORING && !SuperStructure.hasGamePiece()
        && SuperStructure.isDistanceToCloseArm()).onTrue(Do(() -> setIDLE()));

    new Trigger(() -> currentRobotState == RobotConstants.SCORING && SuperStructure.isDitancetToOpenSystems())
        .onTrue(Do(() -> setSCORING()));

    // Ball Removing
    new Trigger(() -> driverController.getR2Button() && !SuperStructure.hasGamePiece())
        .onTrue(Do(() -> setBALLREMOVING())); // Not too close

    new Trigger(() -> currentRobotState == RobotConstants.BALLREMOVING && SuperStructure.isFine
        && SuperStructure.isDistanceToEndBallRemove())
        .onTrue(Do(() -> setHOLDBALL()));

    // Climb
    new Trigger(() -> driverController.getSquareButton() && currentRobotState == RobotConstants.IDLE)
        .onTrue(Do(() -> setCLIMB()));

    new Trigger(() -> driverController.getSquareButton() && climb.getTargetState() == ClimbConstants.ALIGN
        && climb.atAlignAngle()).onTrue(Do(() -> climb.setTargetState(ClimbConstants.CLIMB)));

    // IDLE
    new Trigger(() -> driverController.getTouchpadButton()).onTrue(Do(() ->
    setIDLE()));

    // Auto Scoring
    new Trigger(() -> driverController.getRawButton(9)).onTrue(Do(() -> SuperStructure.toggleAutoScoring()));

    // Scoring Levels
    new Trigger(() -> operatorController.getPOV() == 180)
        .onTrue(Do(() -> SuperStructure.setScoringPreset(ScoringLevel.L1)));

    new Trigger(() -> operatorController.getPOV() == 0)
        .onTrue(Do(() -> SuperStructure.setScoringPreset(ScoringLevel.L4)));

    new Trigger(() -> operatorController.getPOV() == 270)
        .onTrue(Do(() -> SuperStructure.setScoringPreset(ScoringLevel.L3)));

    new Trigger(() -> operatorController.getPOV() == 90)
        .onTrue(Do(() -> SuperStructure.setScoringPreset(ScoringLevel.L2)));

    // Skyhook
    new Trigger(() -> driverController.getCircleButton() && currentRobotState == RobotConstants.HOLDBALL)
        .onTrue(Do(() -> setSKYHOOK()));

    new Trigger(() -> driverController.getCrossButton()).onTrue(Do(() -> setEject()));


    //Manuels
    new Trigger(() -> operatorController.getRightBumperButton() || operatorController.getLeftBumperButton())
    .onTrue(Do(() -> intake.setSystemFunctionState(StatesConstants.MANUEL)))
    .onFalse(Do(() -> intake.setSystemFunctionState(StatesConstants.AUTOMATIC)));

    new Trigger(() -> Math.abs(operatorController.getLeftY() ) < 0.05 )
    .onTrue(Do(() -> elevator.setSystemFunctionState(StatesConstants.MANUEL)))
    .onFalse(Do(() -> elevator.setSystemFunctionState(StatesConstants.AUTOMATIC)));

    new Trigger(() -> Math.abs(operatorController.getRightY() ) < 0.05 )
    .onTrue(Do(() -> arm.setSystemFunctionState(StatesConstants.MANUEL)))
    .onFalse(Do(() -> arm.setSystemFunctionState(StatesConstants.AUTOMATIC)));

  }

  private static Command Do(Runnable toRun) {
    return CommandUtil.instantOf(toRun);
  }

}