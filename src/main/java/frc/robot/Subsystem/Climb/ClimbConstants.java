
package frc.robot.Subsystem.Climb;


import com.ma5951.utils.RobotControl.StatesTypes.State;
import com.ma5951.utils.RobotControl.StatesTypes.StatesConstants;

import frc.robot.Robot;
import frc.robot.Subsystem.Climb.IOs.ClimbIO;
import frc.robot.Subsystem.Climb.IOs.ClimbIOReal;
import frc.robot.Subsystem.Climb.IOs.ClimbIOSim;

public class ClimbConstants {

    public static final double kCURRENT_LIMIT = 30;
    public static final double kCONTINUOUS_LOWER_LIMIT = 25;
    public static final double kCONTINUOUS_CURRENT_TIME = 0.1;
    public static final Boolean kENABLE_CURRENT_LIMIT = false;
    
    public static final double GEAR = 192;
    public static final double TOLERANCE = 10;

    public static final double FREE_POSITION = 0.15;
    public static final double LOCK_POSITION = 0;

    public static final double MAX_ANGLE = 0;
    public static final double MIN_ANGLE = -200;
    public static final double ALIGN_ANGLE = -220;
    public static final double CLIMB_ANGLE = -63;

    public static final double IDLE_VOLTAGE = 3;
    public static final double ALIGN_VOLTAGE = -3;
    public static final double CLIMB_VOLTAGE = 1.5;

    public static final double MANUEL_VOLTAGE_LIMIT = 0;
    public static final double HOME_VOLTAGE = 0;

    public static final State IDLE = StatesConstants.IDLE;
    public static final State ALIGN = new State("ALIGN");
    public static final State CLIMB = new State("CLIMB");
    public static final State OPEN_RACHET = new State("OPEN_RACHET");

    public static final State[] SUBSYSTEM_STATES = new State[] {IDLE , ALIGN , CLIMB, OPEN_RACHET};

    public static ClimbIO getClimbIO() {
        if (Robot.isReal()) {
            return new ClimbIOReal();
        } 

        return new ClimbIOSim();
    }
}


