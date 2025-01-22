
package frc.robot.Subsystem.Elevator;

import com.ma5951.utils.RobotControl.StatesTypes.State;
import com.ma5951.utils.RobotControl.StatesTypes.StatesConstants;

import frc.robot.Robot;
import frc.robot.Subsystem.Elevator.IOs.ElevatorIO;
import frc.robot.Subsystem.Elevator.IOs.ElevatorIOReal;
import frc.robot.Subsystem.Elevator.IOs.ElevatorIOSim;

public class ElevatorConstants {

    public static final double MAX_HIGHT = 2.2;
    public static final double MIN_HIGHT = 0;
    public static final double HIGHT_L2 = 1;
    public static final double HIGHT_L3 = 1.3;
    public static final double HIGHT_L1 = 0.4;
    public static final double HIGHT_L4 = 2;
    public static final double HIGHT_INTAKE_CORAL = 1;
    public static final double HIGHT_EJECT_BALL_LOW = 1.1;
    public static final double HIGHT_EJECT_BALL_HIGH = 1.4;
    public static final double HIGHT_PROSESOR = 0;
    public static final double HIGHT_ZERO = 0;

    public static final double GEAR = (82d / 11d) / 2d;
    public static final double SPROKET_PITCH_DIAMETER = 0.0444754;
    public static final double SPROKET_CIRCUMFERENCE = SPROKET_PITCH_DIAMETER * Math.PI;

    public static final int CONTROL_SLOT = 0;
    public static final double kP = 0.2;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double TOLORANCE = 0.02;
    public static final double FEED_FORWARD = 0;
    
    public static final double HOME_VOLTAGE = -3;
    public static final double HOME_CURRENT = 30;
    public static final double CAN_MOVE_CURRENT_LIMIT = 150;
    public static final double MANUEL_VOLTAGE_LIMIT = 1;
    
    public static final double ACCELERATION = 10;
    public static final double CRUSIE_VELOCITY = 52;
    public static final double JERK = 0;

    public static final double WEIGHT_OF_MOVING_PARTS = 9;

    public static final double CURRENT_LIMIT = 40;
    public static final double CONTINUOUS_LOWER_LIMIT = 20; 
    public static final double CONTINUOUS_CURRENT_DURATION = 0.1;
    public static final boolean ENABLE_CURRENT_LIMIT = true;

    public static final State IDLE = StatesConstants.IDLE;
    public static final State HOLD = new State("HOLD");
    public static final State HOME = new State("HOME");
    public static final State INTAKE = new State("INTAKE");
    public static final State SCORING = new State("SCORING");
    public static final State CLIMB = new State("SORTING");
    public static final State BALLREMOVING  = new State("BALLREMOVING ");

    public static final State[] SUBSYSTEM_STATES = new State[] {IDLE , HOLD , INTAKE , SCORING , CLIMB , BALLREMOVING , HOME};

    public static final ElevatorIO getElevatorIO() {
        if (Robot.isReal()) {
            return new ElevatorIOReal();
        }

        return new ElevatorIOSim();
    }
 
}
