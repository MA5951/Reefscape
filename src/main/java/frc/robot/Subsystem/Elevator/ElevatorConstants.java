
package frc.robot.Subsystem.Elevator;

import com.ma5951.utils.RobotControl.StatesTypes.State;
import com.ma5951.utils.RobotControl.StatesTypes.StatesConstants;

import frc.robot.Robot;
import frc.robot.Subsystem.Elevator.IOs.ElevatorIO;
import frc.robot.Subsystem.Elevator.IOs.ElevatorIOReal;
import frc.robot.Subsystem.Elevator.IOs.ElevatorIOSim;

public class ElevatorConstants {

    public static final double MAX_HIGHT = 1.485;
    public static final double MIN_HIGHT = -0.003;
    public static final double HIGHT_L2 = 0.52;//0.49
    public static final double HIGHT_L3 = 0.85;
    public static final double HIGHT_L1 = 0.13;
    public static final double HIGHT_L4 = 1.48;
    public static final double HIGHT_INTAKE_CORAL = 0.59;
    public static final double HIGHT_EJECT_BALL_LOW = 0.25;
    public static final double HIGHT_EJECT_BALL_HIGH = 0.7;
    public static final double HIGHT_PROSESOR = 0;
    public static final double HIGHT_ZERO = 0;

    public static final double SKYHOOK_STOP_HIGHT = 1.3;
    public static final double SKYHOOK_VOLTAGE = 7;

    public static final double GEAR = (82d / 12d) /2d;
    public static final double SPROKET_PITCH_DIAMETER = 0.0444754;
    public static final double SPROKET_CIRCUMFERENCE = SPROKET_PITCH_DIAMETER * Math.PI;

    public static final int CONTROL_SLOT = 0;
    public static final double kP = 1.1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double TOLORANCE = 0.01;
    public static final double FEED_FORWARD = 0.42;
    
    public static final double HOME_VOLTAGE = -1;
    public static final double HOME_CURRENT = -30;
    public static final double HOME_TIME = 0.2;
    public static final double CAN_MOVE_CURRENT_LIMIT = 150;
    public static final double MANUEL_VOLTAGE_LIMIT = 6;
    
    public static final double ACCELERATION = 30 / SPROKET_CIRCUMFERENCE;
    public static final double CRUSIE_VELOCITY = 14 / SPROKET_CIRCUMFERENCE;
    public static final double JERK = 0;

    public static final double WEIGHT_OF_MOVING_PARTS = 9;

    public static final double CURRENT_LIMIT = 40;
    public static final double CONTINUOUS_LOWER_LIMIT = 20; 
    public static final double CONTINUOUS_CURRENT_DURATION = 0.1;
    public static final boolean ENABLE_CURRENT_LIMIT = false;

    public static final State IDLE = StatesConstants.IDLE;
    public static final State HOLD = new State("HOLD");
    public static final State HOME = new State("HOME");
    public static final State INTAKE = new State("INTAKE");
    public static final State SCORING = new State("SCORING");
    public static final State CLIMB = new State("SORTING");
    public static final State BALLREMOVING  = new State("BALLREMOVING");
    public static final State SKYHOOK = new State("SKYHOOK");

    public static final State[] SUBSYSTEM_STATES = new State[] {IDLE , HOLD , INTAKE , SCORING , CLIMB , BALLREMOVING , HOME , SKYHOOK};

    public static final ElevatorIO getElevatorIO() {
        if (Robot.isReal()) {
            return new ElevatorIOReal();
        }

        return new ElevatorIOSim();
    }
 
}
