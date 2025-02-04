package frc.robot.Subsystem.Arm;

import com.ma5951.utils.RobotControl.StatesTypes.State;
import com.ma5951.utils.RobotControl.StatesTypes.StatesConstants;

import frc.robot.Robot;
import frc.robot.Subsystem.Arm.IOs.ArmIO;
import frc.robot.Subsystem.Arm.IOs.ArmIOReal;
import frc.robot.Subsystem.Arm.IOs.ArmIOSim;

public class ArmConstants {

    public static final double MAX_ANGLE = 180; 
    public static final double MIN_ANGLE = 0;  
    public static final double PARALLEL_ANGLE = 90;
    public static final double MAX_ANGLE_BALL = 110; 
    public static final double ZERO_ANGLE = 10;

    public static final double ANGLE_L1 = 27;
    public static final double ANGLE_L2 = 124;
    public static final double ANGLE_L3 = 140;
    public static final double ANGLE_L4 = 135;
    public static final double ANGLE_OFFSET_L4_AFTER_EJECT = 14;

    public static final double INTAKE_CORALS_ANGLE = 32;
    public static final double EJECT_BALL_START_ANGLE = 0;
    public static final double EJECT_BALL_STOP_ANGLE = 130;

    public static final double SKYHOOK_START_ANGLE = 96;
    public static final double SKYHOOK_START_HIGHT = 1.2;
    public static final double SKYHOOK_END_ANGLE = 180;
    
    public static final double HOLD_ANGLE = 145;

    public static final double ABS_ENCODER_OFFSET = 0;

    public static final double GEAR_RATIO = 18;
    public static final double CANCODER_RATIO = 3;

    public static final double kP = 25;//35
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double FEED_FORWARD_VOLTAGE = 0.085;

    public static final int CONTROL_SLOT = 0;
    public static final double TOLERANCE = 2;

    public static final double PEAK_CURRENT_LIMIT = 7;
    public static final double CONTINUOUS_CURRENT_LIMIT = 6;
    public static final double CONTINUOUS_CURRENT_DURATION = 0.05;
    public static final boolean ENABLE_CURRENT_LIMIT = true;

    public static final double kCAN_MOVE_CURRENT_LIMIT = 25;
    public static final double kMANUEL_VOLTAGE_LIMIT = 5;

    public static final State IDLE = StatesConstants.IDLE;
    public static final State HOLD = new State("HOLD");
    public static final State INTAKE = new State("INTAKE");
    public static final State SCORING = new State("SCORING");
    public static final State BALLREMOVING = new State("BALLREMOVING");
    public static final State SKYHOOK = new State("SKYHOOK");

    public static final State[] SUBSYSTEM_STATES = new State[] {IDLE, HOLD, INTAKE, SCORING, BALLREMOVING , SKYHOOK};

    public static ArmIO getArmIO() {
        if (Robot.isReal()) {
            return new ArmIOReal();
        } else {
            return new ArmIOSim();
        }
    }
}
