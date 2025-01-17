
package frc.robot;

import org.ironmaple.simulation.SimulatedArena;

import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Logger.LoggedInt;
import com.ma5951.utils.Logger.LoggedPose2d;
import com.ma5951.utils.Logger.MALog;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotControl.Field;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveConstants;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  public static boolean isStartingPose = false;
  private LoggedPose2d simulationPose2d;
  private MALog maLog;
  private LoggedInt Log;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    simulationPose2d = new LoggedPose2d("/Simulation/Pose");
    maLog = MALog.getInstance(RobotConstants.COMP_LOG);
    Log = new LoggedInt("/SuperStructure/Closest Tag");

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    PoseEstimator.getInstance().update();
    m_robotContainer.updatePeriodic();

    Field.setAllianceReefFaces();

    Log.update(Field.getClosestFace(PoseEstimator.getInstance().getEstimatedRobotPose()).TagID());
  }

  @Override
  public void disabledInit() {
    maLog.stopLog();
    ;
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.updateDisablePeriodic();
  }

  @Override
  public void autonomousInit() {
    maLog.startAutoLog();
    m_robotContainer.updateAutoInit();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

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
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    m_robotContainer.configureTeleopCommands();
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
