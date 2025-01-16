
package frc.robot.RobotControl;

import java.util.HashMap;
import java.util.function.Supplier;


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
        LEFT(0 , 0),
        RIGHT(0 , 0),
        NONE(0 , 0);

        public final double tX;
        public final double distanceFromTag;

        ScoringLocation(double Tx , double DistanceFromTag) {
            tX = Tx;
            distanceFromTag = DistanceFromTag;
        }
    }

    public static enum BallHight {
        HIGH,
        LOW
    }

    private HashMap<Integer, String> reefFaces = new HashMap<Integer, String>();


}