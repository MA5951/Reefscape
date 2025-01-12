
package frc.robot;

import java.util.spi.CurrencyNameProvider;

import com.ma5951.utils.RobotControl.DeafultRobotContainer;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotControl.Field;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Arm.ArmConstants;
import frc.robot.Subsystem.Intake.Intake;
import frc.robot.Subsystem.Intake.IntakeConstants;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveAutoFollower;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;
import frc.robot.Subsystem.Vision.Vision;
import frc.robot.commands.Swerve.TeleopSwerveController;
import frc.robot.commands.deafultCommands.IntakeDeafultCommand;

public class RobotContainer extends DeafultRobotContainer {

  public static SwerveSubsystem swerve;
  public static Vision vision;
  public static PoseEstimator poseEstimator;
  public static SwerveAutoFollower swerveAutoFollower;
  public static Intake intake;
  public static Arm arm;

  public RobotContainer() {
    super(
        PortMap.Controllers.driveID,
        PortMap.Controllers.operatorID,
        PortMap.Controllers.driveRumbleID,
        PortMap.Controllers.operatorRumbleID);
    // swerve = SwerveSubsystem.getInstance();
    // vision = Vision.getInstance();
    // poseEstimator = PoseEstimator.getInstance();
    // swerveAutoFollower = SwerveAutoFollower.getInstance();
    intake = Intake.getInstance();
    arm = Arm.getInstance();

    configureBindings();
    setUpAutoCommands();

  }

  public void configureTeleopCommands() {
    // CommandScheduler.getInstance().setDefaultCommand(swerve.getInstance(),
    // new TeleopSwerveController(driverController));
    CommandScheduler.getInstance().setDefaultCommand(intake,
        new IntakeDeafultCommand());

  }

  public void setUpAutoCommands() {
    // setAutoOptions(null);
  }

  public void setIDLE() {
    setCurrentState(RobotConstants.IDLE);
    intake.setTargetState(IntakeConstants.IDLE);
    arm.setTargetState(ArmConstants.IDLE);
  }

  public void setINTAKE() {
    setCurrentState(RobotConstants.INTAKE);
    intake.setTargetState(IntakeConstants.INTAKE);
    arm.setTargetState(ArmConstants.INTAKE);
  }

  public void setScoring(Field.ScoringLevel ScoringLevel) {
    RobotConstants.SUPER_STRUCTURE.setScoringPreset(ScoringLevel);
    setCurrentState(RobotConstants.SCORING);
    intake.setTargetState(IntakeConstants.SCORING);
    arm.setTargetState(ArmConstants.SCORING);
  }

  private void configureBindings() {

    // Update Offset
    new Trigger(() -> driverController.getTriangleButton())
        .onTrue(new InstantCommand(() -> TeleopSwerveController.driveController.updateDriveHeading()));

    // Manuel Vision Update
    new Trigger(() -> driverController.getPSButton())
        .onTrue(new InstantCommand(() -> Vision.getInstance().updateOdometry()));

    // Intake
    // new Trigger(() -> driverController.getR1Button())
    //     .onTrue(new InstantCommand(() -> RobotConstants.SUPER_STRUCTURE.setIntakeHight(0))
    //         .andThen(new InstantCommand(() -> setINTAKE())));

            new Trigger(() -> driverController.getR1Button()).onTrue(new InstantCommand(() -> setINTAKE()));

    new Trigger(() -> currentRobotState == RobotConstants.INTAKE
        && RobotConstants.SUPER_STRUCTURE.getGamePiece() == Field.GamePiece.CORAL
        && intake.getRearSensor())
        .onTrue(new InstantCommand(() -> setIDLE()));

    new Trigger(() -> currentRobotState == RobotConstants.INTAKE
        && RobotConstants.SUPER_STRUCTURE.getGamePiece() == Field.GamePiece.BALL)
        .onTrue(new InstantCommand(() -> setIDLE()));
  }

}