
package frc.robot.RobotControl;

import java.util.HashMap;
import java.util.function.Supplier;

import com.ma5951.utils.Utils.DriverStationUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotConstants;
import frc.robot.Utils.ReefFace;

public class Field {

    public static enum GamePiece {
        CORAL(() -> 0d),
        BALL(() -> 0d),
        NONE(() -> 0d);

        public final Supplier<Double> holdValue;

        GamePiece(Supplier<Double> HoldValue) {
            holdValue = HoldValue;
        }

    }

    public static enum ScoringLevel {
        L1(0, 10, 4),
        L2(0, 10, 5),
        L3(0, 10, 1),
        L4(0, 10, 1);

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
        LEFT(0, 0),
        RIGHT(0, 0),
        NONE(0, 0);

        public final double tX;
        public final double distanceFromTag;

        ScoringLocation(double Tx, double DistanceFromTag) {
            tX = Tx;
            distanceFromTag = DistanceFromTag;
        }
    }

    public static enum BallHight {
        HIGH(0),
        LOW(0),
        NONE(0);

        public final double elevatorHight;

        BallHight(double ElevatorHight) {
            elevatorHight = ElevatorHight;
        }
    }

    private static HashMap<Integer, ReefFace> ReefFaces = new HashMap<Integer, ReefFace>();
    private static ReefFace blankFace = new ReefFace(0, 0, BallHight.NONE, new Pose2d());
    private static double diff = 0;
    private static ReefFace closestFace;
    private static double closestDistance;
    private static double spatialDistance;
    private static double angleDiff = 0;
    private static double totalDistance;
    private static boolean wasReefSet = false;

    public Field() {

    }

    public static void setAllianceReefFaces() {
        if (!wasReefSet && DriverStation.getAlliance().isPresent()) {
            wasReefSet = true;
            if (DriverStationUtil.getAlliance() == Alliance.Blue) {
                // Blue Side Reef
                ReefFaces.put(17, new ReefFace(60, 17, Field.BallHight.LOW, RobotConstants.Tag17Pose));
                ReefFaces.put(22, new ReefFace(120, 22, Field.BallHight.HIGH, RobotConstants.Tag22Pose));
                ReefFaces.put(20, new ReefFace(-120, 20, Field.BallHight.HIGH, RobotConstants.Tag20Pose));
                ReefFaces.put(19, new ReefFace(-60, 19, Field.BallHight.LOW, RobotConstants.Tag19Pose));
                ReefFaces.put(18, new ReefFace(0, 18, Field.BallHight.HIGH, RobotConstants.Tag18Pose));
                ReefFaces.put(21, new ReefFace(180, 21, Field.BallHight.LOW, RobotConstants.Tag21Pose));
            } else {
                // Red Side Reef
                ReefFaces.put(10, new ReefFace(0, 10, Field.BallHight.LOW, RobotConstants.Tag10Pose));
                ReefFaces.put(7, new ReefFace(180, 7, Field.BallHight.HIGH, RobotConstants.Tag7Pose));
                ReefFaces.put(11, new ReefFace(60, 11, Field.BallHight.HIGH, RobotConstants.Tag11Pose));
                ReefFaces.put(9, new ReefFace(-60, 9, Field.BallHight.HIGH, RobotConstants.Tag9Pose));
                ReefFaces.put(8, new ReefFace(-120, 8, Field.BallHight.LOW, RobotConstants.Tag8Pose));
                ReefFaces.put(6, new ReefFace(120, 6, Field.BallHight.LOW, RobotConstants.Tag6Pose));
            }
        }
    }

    public static ReefFace getFaceByID(int TagID) {
        if (wasReefSet) {
            return ReefFaces.get(TagID);
        }

        return blankFace;
    }

    public static ReefFace getClosestFace(Pose2d robotPose) {
        if (wasReefSet) {
            closestFace = null;
            closestDistance = Double.MAX_VALUE;

            for (ReefFace face : ReefFaces.values()) {

                spatialDistance = euclideanDistance(robotPose, face.tagPose());
                angleDiff = angularDistance(robotPose, face.tagPose());
                totalDistance = spatialDistance  + angleDiff ;

                if (totalDistance < closestDistance) {
                    closestDistance = totalDistance;
                    closestFace = face;
                }
            }

            return closestFace;
        }

        return blankFace;
    }

    public static double angularDistance(Pose2d pose1, Pose2d pose2) {
        diff = Math.abs(pose1.getRotation().getRadians() - pose2.getRotation().getRadians());
        return Math.min(diff, 2 * Math.PI - diff); 
    }

    public static double euclideanDistance(Pose2d pose1, Pose2d pose2) {
        return Math.sqrt(Math.pow(pose1.getX() - pose2.getX(), 2) + Math.pow(pose1.getY() - pose2.getY(), 2));
    }

}