
package frc.robot.RobotControl;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotConstants;
import frc.robot.Utils.ReefFace;

public class Field {

    public static enum GamePiece {
        CORAL(() -> 0d),//TODO add a real value
        BALL(() -> 0d), //TODO add a real value
        NONE(() -> 0d); //TODO add a real value

        public final Supplier<Double> holdValue;

        GamePiece(Supplier<Double> HoldValue) {
            holdValue = HoldValue;
        }

    }

    public static enum ScoringLevel {
        L1(0, 10, 4), //TODO add a real value

        L2(0, 10, 5),//TODO add a real value

        L3(0, 10, 1),//TODO add a real value

        L4(0, 10, 1);//TODO add a real value


        public final double hight;
        public final double angle;
        public final double ejectVolt;

        ScoringLevel(double ScoringHight, double ScoringAngle, double EjectVolt) {
            hight = ScoringHight;
            angle = ScoringAngle;
            ejectVolt = EjectVolt;
        }
    }

    public static enum ScoringLocation {
        LEFT(0, 0),  //TODO add a real value
        RIGHT(0, 0), //TODO add a real value
        NONE(0, 0);

        public final double tX;
        public final double distanceFromTag;

        ScoringLocation(double Tx, double DistanceFromTag) {
            tX = Tx;
            distanceFromTag = DistanceFromTag;
        }
    }

    public static enum BallHight {
        HIGH(0), //TODO add a real value
        LOW(0), //TODO add a real value
        NONE(0); //TODO add a real value

        public final double elevatorHight;

        BallHight(double ElevatorHight) {
            elevatorHight = ElevatorHight;
        }
    }

    private static HashMap<Integer, ReefFace> ReefFaces = new HashMap<Integer, ReefFace>();
    private static ReefFace blankFace = new ReefFace(0, 0, BallHight.NONE, new Pose2d());
    public final static List<Integer> ReefTags = new ArrayList<>();
    private static ReefFace closestFace;
    private static double closestDistanceReef;
    private static double spatialDistanceReef;
    private static boolean wasFieldSet = false;


    public Field() {

    }

    public static void setAllianceReefFaces(Alliance alliance) {
        if (!wasFieldSet) {
            wasFieldSet = true;
            if (alliance == Alliance.Blue) {
                // Blue Side
                ReefFaces.put(17, new ReefFace(RobotConstants.Tag17Pose.getRotation().getDegrees(), 17, Field.BallHight.LOW, RobotConstants.Tag17Pose));
                ReefFaces.put(22, new ReefFace(RobotConstants.Tag22Pose.getRotation().getDegrees(), 22, Field.BallHight.HIGH, RobotConstants.Tag22Pose));
                ReefFaces.put(20, new ReefFace(RobotConstants.Tag20Pose.getRotation().getDegrees(), 20, Field.BallHight.HIGH, RobotConstants.Tag20Pose));
                ReefFaces.put(19, new ReefFace(RobotConstants.Tag19Pose.getRotation().getDegrees(), 19, Field.BallHight.LOW, RobotConstants.Tag19Pose));
                ReefFaces.put(18, new ReefFace(RobotConstants.Tag18Pose.getRotation().getDegrees(), 18, Field.BallHight.HIGH, RobotConstants.Tag18Pose));
                ReefFaces.put(21, new ReefFace(RobotConstants.Tag21Pose.getRotation().getDegrees(), 21, Field.BallHight.LOW, RobotConstants.Tag21Pose));
                ReefTags.add(17);
                ReefTags.add(22);
                ReefTags.add(20);
                ReefTags.add(19);
                ReefTags.add(18);
                ReefTags.add(21);
            } else {
                // Red Side
                ReefFaces.put(10, new ReefFace(RobotConstants.Tag10Pose.getRotation().getDegrees(), 10, Field.BallHight.LOW, RobotConstants.Tag10Pose));
                ReefFaces.put(7, new ReefFace(RobotConstants.Tag7Pose.getRotation().getDegrees(), 7, Field.BallHight.HIGH, RobotConstants.Tag7Pose));
                ReefFaces.put(11, new ReefFace(RobotConstants.Tag11Pose.getRotation().getDegrees(), 11, Field.BallHight.HIGH, RobotConstants.Tag11Pose));
                ReefFaces.put(9, new ReefFace(RobotConstants.Tag9Pose.getRotation().getDegrees(), 9, Field.BallHight.HIGH, RobotConstants.Tag9Pose));
                ReefFaces.put(8, new ReefFace(RobotConstants.Tag8Pose.getRotation().getDegrees(), 8, Field.BallHight.LOW, RobotConstants.Tag8Pose));
                ReefFaces.put(6, new ReefFace(RobotConstants.Tag6Pose.getRotation().getDegrees(), 6, Field.BallHight.LOW, RobotConstants.Tag6Pose));
                ReefTags.add(10);
                ReefTags.add(7);
                ReefTags.add(11);
                ReefTags.add(9);
                ReefTags.add(8);
                ReefTags.add(6);
            }
        }
    }

    public static boolean isReefTag(int TagID) {
        if (wasFieldSet) {
            return ReefTags.contains(TagID);
        } else {
            return false;
        }
    }

    public static ReefFace getFaceByID(int TagID) {
        if (wasFieldSet) {
            return ReefFaces.get(TagID);
        }

        return blankFace;
    }

    public static ReefFace getClosestReefFace(Pose2d robotPose) {
        if (wasFieldSet) {
            closestFace = null;
            closestDistanceReef = Double.MAX_VALUE;

            for (ReefFace face : ReefFaces.values()) {

                spatialDistanceReef = euclideanDistance(robotPose, face.tagPose());

                if (spatialDistanceReef < closestDistanceReef) {
                    closestDistanceReef = spatialDistanceReef;
                    closestFace = face;
                }
            }

            return closestFace;
        }

        return blankFace;
    }

    public static double euclideanDistance(Pose2d pose1, Pose2d pose2) {
        return Math.sqrt(Math.pow(pose1.getX() - pose2.getX(), 2) + Math.pow(pose1.getY() - pose2.getY(), 2));
    }
}