
package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.RobotConstants;
import frc.robot.RobotControl.Field;



public record ReefFace(double AbsAngle, int TagID, Field.BallHight BallHight , Pose2d tagPose) { 

        public Pose2d getAlignPose() {
            return tagPose.plus(new Transform2d(-RobotConstants.DistanceToAlign, 0, tagPose.getRotation()));
        }

        public Pose2d getLeftAlignPose() {
            return tagPose.plus(new Transform2d((-0.86/2) + 0.01,0.22, tagPose.getRotation()));
        }

        public Pose2d getRightAlignPose() {
            return tagPose.plus(new Transform2d((-0.86/2) + 0.01, -0.1, tagPose.getRotation()));
        }

        public Pose2d getLeftSemiAlignPose() {
            return tagPose.plus(new Transform2d((-0.86/2) - 0.1,0.22, tagPose.getRotation()));
        }

        public Pose2d getRightSemiAlignPose() {
            return tagPose.plus(new Transform2d((-0.86/2) - 0.1, -0.1, tagPose.getRotation()));
        }

        public Pose2d getBallRemovingPose() {
            return tagPose.plus(new Transform2d((-0.86/2) , 0.1, tagPose.getRotation()));
        }
    }