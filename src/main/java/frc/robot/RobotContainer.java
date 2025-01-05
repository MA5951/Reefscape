
package frc.robot;


import com.ma5951.utils.RobotControl.DeafultRobotContainer;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveAutoFollower;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;
import frc.robot.Subsystem.Vision.Vision;
import frc.robot.commands.Swerve.TeleopSwerveController;

public class RobotContainer extends DeafultRobotContainer{


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

  private void configureBindings() {

    //Update Offset
    new Trigger(() -> driverController.getTriangleButton()).onTrue(new InstantCommand(() -> TeleopSwerveController.driveController.updateDriveHeading()));

    //Manuel Vision Update
    new Trigger(() -> driverController.getPSButton()).onTrue(new InstantCommand(() -> Vision.getInstance().updateOdometry()));

    


  }

}