
package frc.robot.RobotControl;

public class Field {

    public enum GamePiece {
        CORAL(0),
        BALL(0),
        NONE(0);

        public final double holdValue;

        GamePiece(double HoldValue) {
            holdValue = HoldValue;
        }

    }

    public enum ScoringLevel {
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

}
