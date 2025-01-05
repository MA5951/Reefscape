
package frc.robot.Subsystem.Swerve;



import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.ma5951.utils.Logger.LoggedPose2d;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;

public class SwerveAutoFollower {
    private static SwerveAutoFollower swerveAutoFollower;


    private SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    private PoseEstimator poseEstimate = PoseEstimator.getInstance();
    private LoggedPose2d targetPoseLog;
    private LoggedPose2d currentPoseLog;
    private RobotConfig config;

    private SwerveAutoFollower() {
        targetPoseLog = new LoggedPose2d("/Auto/Target Pose");
        currentPoseLog = new LoggedPose2d("/Auto/Current Pose");
        
        try{
            config = RobotConfig.fromGUISettings();
          } catch (Exception e) {
              e.printStackTrace();
          }

        AutoBuilder.configure(
            () -> poseEstimate.getEstimatedRobotPose() ,
            pose -> poseEstimate.resetPose(pose), 
            () -> swerve.getRobotRelativeSpeeds(),
            (speeds , feedforwards) -> swerve.drive(speeds , feedforwards),
           SwerveConstants.getHolonomicController(),
                config,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
              }, swerve);


        Pathfinding.setPathfinder(new LocalADStar());
        PathPlannerLogging.setLogTargetPoseCallback(pose -> targetPoseLog.update(pose));
        PathPlannerLogging.setLogCurrentPoseCallback(pose -> currentPoseLog.update(pose));

        
    }

    public static Command buildAuto(String autoName) {
        return new PathPlannerAuto(autoName);
    }

    public static Command followPath(String path) {
        try {
            return AutoBuilder.followPath(PathPlannerPath.fromPathFile(path));
        } catch (FileVersionException | IOException | ParseException e) {
            e.printStackTrace();
        }
        return new InstantCommand();
    }

    public static Command pathFindToPose(Pose2d targetPose , PathConstraints constraints) {
        return AutoBuilder.pathfindToPose(targetPose, constraints);
    }

    public static SwerveAutoFollower getInstance() {
        if (swerveAutoFollower == null) {
            swerveAutoFollower = new SwerveAutoFollower();  
        }
        return swerveAutoFollower;
      }
}
