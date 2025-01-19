
package frc.robot;

import org.ironmaple.simulation.SimulatedArena;

import com.ma5951.utils.Logger.LoggedPose2d;
import com.ma5951.utils.Logger.MALog;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveConstants;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static boolean isStartingPose = false;
  private LoggedPose2d simulationPose2d;
  private MALog maLog;

  @Override
  public void robotInit() {
    simulationPose2d = new LoggedPose2d("/Simulation/Pose");
    maLog = MALog.getInstance(RobotConstants.COMP_LOG);
    //TODO must do new to RobotContainer!!!! 
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    PoseEstimator.getInstance().update();
    RobotContainer.setAllianceData();
    RobotContainer.updatePeriodic();
    //TODO call the robot container init func

  }

  @Override
  public void disabledInit() {
    //TODO change the robot state to idel 
    maLog.stopLog();
  }

  @Override
  public void disabledPeriodic() {
    RobotContainer.updateDisablePeriodic();
  }

  @Override
  public void autonomousInit() {
    maLog.startAutoLog();
    RobotContainer.updateAutoInit();
    m_autonomousCommand = RobotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    maLog.startTeleopLog();
    //TODO change the robot state to idel 
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    RobotContainer.configureTeleopCommands(); //move up to auto init we use tha smae commands their
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    maLog.startTestLog();
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
    SimulatedArena.getInstance().addDriveTrainSimulation(SwerveConstants.SWERVE_DRIVE_SIMULATION);
  }

  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();
    simulationPose2d.update(SwerveConstants.SWERVE_DRIVE_SIMULATION.getSimulatedDriveTrainPose());

  }

}
