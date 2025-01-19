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
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;
import frc.robot.Subsystem.Vision.Vision;
import frc.robot.Utils.MAProfieldPIDController;

@SuppressWarnings("unused")
public class TeleopSwerveController extends Command {

  public static FieldCentricDriveController driveController;
  public static AngleAdjustController angleAdjustController;
  public static AutoAdjustXYController autoAdjustXYController;
  private static ChassisSpeeds driveControllerSpeeds;
  private static ChassisSpeeds angleAdjustControllerSpeeds;
  private static ChassisSpeeds autoAdjustControllerChassisSpeeds;

  private static SwerveSubsystem swerve;
  private static ChassisSpeeds robotSpeeds;
  private static LoggedString xyControllerLog;
  private static LoggedString theathControllerLog;

  public TeleopSwerveController(PS5Controller controller) {
    swerve = SwerveSubsystem.getInstance();

    driveController = new FieldCentricDriveController(controller, () -> controller.getR2Button(),
        0.4, () -> SwerveSubsystem.getInstance().getFusedHeading());

    angleAdjustController = new AngleAdjustController(() -> SwerveSubsystem.getInstance().getFusedHeading());

    autoAdjustXYController = new AutoAdjustXYController(() -> PoseEstimator.getInstance().getEstimatedRobotPose(),
        () -> SwerveSubsystem.getInstance().getFusedHeading());

    xyControllerLog = new LoggedString("/Subsystems/Swerve/Controllers/XY Controller");
    theathControllerLog = new LoggedString("/Subsystems/Swerve/Controllers/Theath Controller");

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    if (RobotContainer.currentRobotState == RobotConstants.INTAKE) {
      driveControllerSpeeds = driveController.update();
      angleAdjustControllerSpeeds = angleAdjustController.update();
      robotSpeeds.vxMetersPerSecond = driveControllerSpeeds.vxMetersPerSecond;
      robotSpeeds.vyMetersPerSecond = driveControllerSpeeds.vyMetersPerSecond;
      robotSpeeds.omegaRadiansPerSecond = angleAdjustControllerSpeeds.omegaRadiansPerSecond;
      xyControllerLog.update("Drive Controller");
      theathControllerLog.update("Angle Controller");
    } else if (RobotContainer.currentRobotState == RobotConstants.SCORING) {
      SuperStructure.updateXYAdjustController(RobotContainer.currentRobotState);
      autoAdjustControllerChassisSpeeds = autoAdjustXYController.update();
      angleAdjustControllerSpeeds = angleAdjustController.update();
      robotSpeeds.vxMetersPerSecond = autoAdjustControllerChassisSpeeds.vxMetersPerSecond;
      robotSpeeds.vyMetersPerSecond = autoAdjustControllerChassisSpeeds.vyMetersPerSecond;
      robotSpeeds.omegaRadiansPerSecond = angleAdjustControllerSpeeds.omegaRadiansPerSecond;
      xyControllerLog.update("XY Controller");
      theathControllerLog.update("Angle Controller");
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
