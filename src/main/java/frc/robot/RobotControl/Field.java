
package frc.robot.RobotControl;

public class Field {

    public enum GamePiece {
        CORAL,
        BALL,
        NONE
    }

    public enum ScoringLevel {
        L1(0, 10, 1),
        L2(0, 10, 1),
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
