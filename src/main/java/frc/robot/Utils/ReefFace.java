
package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.RobotConstants;
import frc.robot.RobotControl.Field;



public record ReefFace(double AbsAngle, int TagID, Field.BallHight BallHight , Pose2d tagPose) { 

        public Pose2d getAlignPose() {
            return tagPose.plus(new Transform2d(-RobotConstants.DistanceToAlign, 0, tagPose.getRotation()));
        }
    }