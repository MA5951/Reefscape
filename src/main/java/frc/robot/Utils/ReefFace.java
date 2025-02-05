
package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.RobotConstants;
import frc.robot.RobotControl.Field;



public record ReefFace(double AbsAngle, int TagID, Field.BallHight BallHight , Pose2d tagPose) { 

        public Pose2d getAlignPose() {
            return tagPose.plus(new Transform2d(-0.5, 0, tagPose.getRotation()));
        }

        public Pose2d getLeftAlignPose() {
            return tagPose.plus(new Transform2d((-0.86/2) + 0.01,0.15, tagPose.getRotation()));
        }

        public Pose2d getRightAlignPose() {
            return tagPose.plus(new Transform2d((-0.86/2) + 0.01, -0.15, tagPose.getRotation()));
        }

        public Pose2d getLeftSemiAlignPose() {
            return tagPose.plus(new Transform2d((-0.86/2) - 0.1,0.15, tagPose.getRotation()));
        }

        public Pose2d getRightSemiAlignPose() {
            return tagPose.plus(new Transform2d((-0.86/2) - 0.1, -0.15, tagPose.getRotation()));
        }

        public Pose2d getBallRemovingPose() {
            return tagPose.plus(new Transform2d((-0.86/2) , 0, tagPose.getRotation()));
        }

        public double getYDistance(Pose2d RobotPose) {
            return Math.abs(tagPose.rotateBy(tagPose.getRotation()).getY() - RobotPose.rotateBy(tagPose.getRotation()).getY());
        }
    }