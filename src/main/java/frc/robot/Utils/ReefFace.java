
package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotControl.Field;

public record ReefFace(
    double AbsAngle, int TagID, Field.BallHight BallHight , Pose2d tagPose) {}