
package frc.robot.RobotControl;

import java.util.function.Supplier;

public class Field {

    public enum GamePiece {
        CORAL(() -> 0d),
        BALL(() -> 0d),
        NONE(() -> 0d);

        public final Supplier<Double> holdValue;

        GamePiece(Supplier<Double> HoldValue) {
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
