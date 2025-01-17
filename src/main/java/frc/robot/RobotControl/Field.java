
package frc.robot.RobotControl;

import java.util.HashMap;
import java.util.function.Supplier;

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
        HIGH(0),
        LOW(0),
        NONE3(0);

        public final double elevatorHight;

        BallHight(double ElevatorHight) {
            elevatorHight = ElevatorHight;
        }
    }

    private HashMap<Integer, ReefFace> ReefFaces = new HashMap<Integer, ReefFace>();

    public Field() {
        

        //Blue Side Reef
        ReefFaces.put(17, new ReefFace(60, 17, Field.BallHight.LOW));
        ReefFaces.put(22, new ReefFace(120, 22, Field.BallHight.HIGH));
        ReefFaces.put(20, new ReefFace(-120, 20, Field.BallHight.HIGH));
        ReefFaces.put(19, new ReefFace(-60, 19, Field.BallHight.LOW));
        ReefFaces.put(18, new ReefFace(0, 18, Field.BallHight.HIGH));
        ReefFaces.put(21, new ReefFace(180, 21, Field.BallHight.LOW));

        //Red Side Reef
        
        
        
        
        

        ReefFaces.put(10, new ReefFace(0, 10, Field.BallHight.LOW));
        ReefFaces.put(7, new ReefFace(180, 7, Field.BallHight.HIGH));
        ReefFaces.put(11, new ReefFace(60, 11, Field.BallHight.HIGH));
        ReefFaces.put(9, new ReefFace(-60, 9, Field.BallHight.HIGH));
        ReefFaces.put(8, new ReefFace(-120, 8, Field.BallHight.LOW));
        ReefFaces.put(6, new ReefFace(120, 6, Field.BallHight.LOW));
        

    }

}