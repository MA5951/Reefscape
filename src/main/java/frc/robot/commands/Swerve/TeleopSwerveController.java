// Copyright (c) FIRST and other WPILib contributors.
// Open Sostatic urce Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import com.ma5951.utils.Logger.LoggedString;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.Field;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;
import frc.robot.Subsystem.Vision.Vision;
import frc.robot.Utils.MAProfieldPIDController;

@SuppressWarnings("unused")
public class TeleopSwerveController extends Command {

  public static FieldCentricDriveController driveController;
  public static AngleAdjustController angleAdjustController;
  public static RelativAngleAdjustController relativAngleAdjustController;
  public static AutoAdjustXYController autoAdjustXYController;
  private ChassisSpeeds driveControllerSpeeds;
  private ChassisSpeeds angleAdjustControllerSpeeds;
  private ChassisSpeeds relativAngleAdjustControllerSpeeds;
  private ChassisSpeeds autoAdjustControllerChassisSpeeds;

  private SwerveSubsystem swerve;
  private ChassisSpeeds robotSpeeds;
  private LoggedString xyControllerLog;
  private LoggedString theathControllerLog;
  private boolean isField;

  public TeleopSwerveController(PS5Controller controller) {
    swerve = SwerveSubsystem.getInstance();

    driveController = new FieldCentricDriveController(controller, () -> controller.getR2Button(),
        0.4, () -> SwerveSubsystem.getInstance().getFusedHeading());
    angleAdjustController = new AngleAdjustController(() -> SwerveSubsystem.getInstance().getFusedHeading(), 60);
    relativAngleAdjustController = new RelativAngleAdjustController(0, () -> Vision.getInstance().getTx());

    autoAdjustXYController = new AutoAdjustXYController(
        () -> PoseEstimator.getInstance().getEstimatedRobotPose());

    autoAdjustXYController.updateSetPoint(new Pose2d(3.59, 2.56, Rotation2d.kZero));

    xyControllerLog = new LoggedString("/Subsystems/Swerve/Controllers/XY Controller");
    theathControllerLog = new LoggedString("/Subsystems/Swerve/Controllers/Theath Controller");

    isField = true;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    angleAdjustController
        .setSetPoint(Field.getClosestFace(PoseEstimator.getInstance().getEstimatedRobotPose()).AbsAngle());

    driveControllerSpeeds = driveController.update();
    autoAdjustControllerChassisSpeeds = autoAdjustXYController.update(isField);

    if (RobotContainer.driverController.getCircleButton()) {

      if (Vision.getInstance().getTagID() == Field.getClosestFace(PoseEstimator.getInstance().getEstimatedRobotPose())
          .TagID()
          && PoseEstimator.getInstance().getEstimatedRobotPose().getTranslation().getDistance(
              Field.getClosestFace(PoseEstimator.getInstance().getEstimatedRobotPose()).getAlignPose()
                  .getTranslation()) < 0.8) {

        autoAdjustXYController.setPID(
            new MAProfieldPIDController(0.4, 0, 0.014, new Constraints(4.9, 6), 0.2, 0),
            new MAProfieldPIDController(0.15, 0, 0.0017, new Constraints(4.9, 6), 0.2, 0));
        autoAdjustXYController.updateSetPoint(new Pose2d(7, 0, Rotation2d.kZero));
        autoAdjustXYController.updateMeaurment(() -> Vision.getInstance().getPoseForRelativReefAlign());
        isField = false;
      } else {
        autoAdjustXYController
            .updateSetPoint(Field.getClosestFace(PoseEstimator.getInstance().getEstimatedRobotPose()).getAlignPose());
      }

      robotSpeeds.omegaRadiansPerSecond = angleAdjustController.update().omegaRadiansPerSecond;
      robotSpeeds.vxMetersPerSecond = autoAdjustControllerChassisSpeeds.vxMetersPerSecond;
      robotSpeeds.vyMetersPerSecond = autoAdjustControllerChassisSpeeds.vyMetersPerSecond;

    } else {
      xyControllerLog.update("Drive Controller");
      theathControllerLog.update("Drive Controller");
      robotSpeeds = driveControllerSpeeds;
    }
    swerve.drive(robotSpeeds);

  }

  @Override
  public void end(boolean interrupted) {
    swerve.setModules(new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    });
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public static boolean atPointForScoring() {
    return false;// TODO atPoint and under speed
  }
}
