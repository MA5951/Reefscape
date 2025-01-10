// Copyright (c) FIRST and other WPILib contributors.
// Open Sostatic urce Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import com.ma5951.utils.Logger.LoggedString;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;
import frc.robot.Subsystem.Vision.Vision;

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

  public TeleopSwerveController(PS5Controller controller) {
    swerve = SwerveSubsystem.getInstance();

    driveController = new FieldCentricDriveController(controller, () -> controller.getR2Button(),
        0.4, () -> SwerveSubsystem.getInstance().getFusedHeading());
    angleAdjustController = new AngleAdjustController(() -> SwerveSubsystem.getInstance().getFusedHeading(), 0);
    relativAngleAdjustController = new RelativAngleAdjustController(0, () -> Vision.getInstance().getTx());
    autoAdjustXYController = new AutoAdjustXYController(() -> Vision.getInstance().getPoseForRelativReefAlign());

    autoAdjustXYController.updateSetPoint(new Pose2d(7, 0, Rotation2d.kZero));

    xyControllerLog = new LoggedString("/Subsystems/Swerve/Controllers/XY Controller");
    theathControllerLog = new LoggedString("/Subsystems/Swerve/Controllers/Theath Controller");

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    driveControllerSpeeds = driveController.update();
    autoAdjustControllerChassisSpeeds = autoAdjustXYController.update();

    if (RobotContainer.currentRobotState == RobotConstants.ALIGN) {
      xyControllerLog.update("XY Controller");
      theathControllerLog.update("0 Controller");
      robotSpeeds.vxMetersPerSecond = autoAdjustControllerChassisSpeeds.vxMetersPerSecond;
      robotSpeeds.vyMetersPerSecond = autoAdjustControllerChassisSpeeds.vyMetersPerSecond;
      robotSpeeds.omegaRadiansPerSecond = 0;
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
}
