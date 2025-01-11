
package frc.robot;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ma5951.utils.RobotControl.DeafultRobotContainer;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveAutoFollower;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;
import frc.robot.Subsystem.Vision.Vision;
import frc.robot.commands.Swerve.TeleopSwerveController;

public class RobotContainer extends DeafultRobotContainer{

  private TalonFX leftMotor;
  private TalonFX rightMotor;

  public RobotContainer() {
    super(
      PortMap.Controllers.driveID, 
      PortMap.Controllers.operatorID, 
      PortMap.Controllers.driveRumbleID,
      PortMap.Controllers.operatorRumbleID);
    SwerveSubsystem.getInstance();
    Vision.getInstance();
    PoseEstimator.getInstance();
    SwerveAutoFollower.getInstance();

    
    
    configureBindings();
    setUpAutoCommands();
    CommandScheduler.getInstance().setDefaultCommand(SwerveSubsystem.getInstance(), new TeleopSwerveController(driverController));
  }

  public void setUpAutoCommands() {
    //setAutoOptions(null);
  }

  public void setIDLE() {
    setCurrentState(RobotConstants.IDLE);
  }

  public void setALIGN() {
    setCurrentState(RobotConstants.ALIGN);
  }

  private void configureBindings() {

    //Update Offset
    new Trigger(() -> driverController.getTriangleButton()).onTrue(new InstantCommand(() -> TeleopSwerveController.driveController.updateDriveHeading()));

    //Manuel Vision Update
    new Trigger(() -> driverController.getPSButton()).onTrue(new InstantCommand(() -> Vision.getInstance().updateOdometry()));

    //Start Align
    new Trigger(() -> driverController.getCircleButton()).whileTrue(new InstantCommand(() -> setALIGN()))
    .whileFalse(new InstantCommand(() -> setIDLE()));
    
    new Trigger(() -> driverController.getCrossButton()).onTrue(new InstantCommand(() -> SwerveSubsystem.getInstance().resetEncoders()));

  }

  @Override
  public Command getAutonomousCommand() {
    return SwerveAutoFollower.pathFindToPose(new Pose2d(3.215, 4.070, Rotation2d.kZero), new PathConstraints(3, 3, 300, 400));
  }

}