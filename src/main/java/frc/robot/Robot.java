
package frc.robot;

import org.ironmaple.simulation.SimulatedArena;

import com.ma5951.utils.Logger.LoggedPose2d;
import com.ma5951.utils.Logger.LoggedPose3d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

import com.ma5951.utils.Logger.MALog;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveConstants;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static boolean isStartingPose = false;
  private LoggedPose2d simulationPose2d;
  private LoggedPose3d scoringPose3d;
  private Pose3d ScoringPose;
  private MALog maLog;

  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  private int firstHue = 0;
  private boolean on;
  private double lastChange;

  @Override
  public void robotInit() {
    @SuppressWarnings("unused")
    RobotContainer mContainer = new RobotContainer();
    simulationPose2d = new LoggedPose2d("/Simulation/Pose");
    scoringPose3d = new LoggedPose3d("/Simulation/Scoring Pose");
    ScoringPose = RobotConstants.SIM_ARM_OFFSET;
    maLog = MALog.getInstance(RobotConstants.COMP_LOG);

    led = new AddressableLED(0);
    ledBuffer = new AddressableLEDBuffer(200);
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    PoseEstimator.getInstance().update();
    RobotContainer.setAllianceData();
    RobotContainer.updatePeriodic();

    smoothWaveColorPattern(3, 1, 1, new Color[] { new Color(255, 237, 70),
        new Color(230, 0, 255), new Color(51, 204, 204) });
    //led.setData(ledBuffer);

  }

  @Override
  public void disabledInit() {
    RobotContainer.setCurrentState(RobotConstants.IDLE);
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
    RobotContainer.setCurrentState(RobotConstants.IDLE);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    RobotContainer.configureTeleopCommands();
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

    ScoringPose = new Pose3d(ScoringPose.getX(), ScoringPose.getY(),
        RobotContainer.elevator.getHight() + RobotConstants.SIM_ARM_OFFSET.getZ(),
        new Rotation3d(0, ConvUtil.DegreesToRadians(-RobotContainer.arm.getPosition()), 0));
    scoringPose3d.update(ScoringPose);

  }

  public void smoothWaveColorPattern(int numColors, double period, double speed, Color[] colors) {
    double elapsedTime = Timer.getFPGATimestamp();

    for (int i = 0; i < ledBuffer.getLength(); i++) {
      double position = ((double) i / ledBuffer.getLength()) + (elapsedTime * speed / period);
      double progress = position - (int) position;

      int startColorIndex = (int) (position % numColors);
      int endColorIndex = (startColorIndex + 1) % numColors;
      Color startColor = colors[startColorIndex];
      Color endColor = colors[endColorIndex];

      Color currentColor = new Color(
          startColor.red + (endColor.red - startColor.red) * progress,
          startColor.green + (endColor.green - startColor.green) * progress,
          startColor.blue + (endColor.blue - startColor.blue) * progress);

      ledBuffer.setLED(i, currentColor);
    }
  }

}
