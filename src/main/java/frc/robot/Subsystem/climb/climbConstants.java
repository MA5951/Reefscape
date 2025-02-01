
package frc.robot.Subsystem.Climb;

import java.lang.instrument.ClassFileTransformer;

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

    public static final double FREE_POSITION = 0;
    public static final double LOCK_POSITION = 0;

    public static final double ALIGN_ANGLE = 0;
    public static final double CLIMB_ANGLE = 0;

    public static final State IDLE = StatesConstants.IDLE;
    public static final State ALIGN = new State("ALIGN");
    public static final State CLIMB = new State("CLIMB");

    public static final State[] SUBSYSTEM_STATES = new State[] {IDLE , ALIGN , CLIMB};

    public static ClimbIO getClimbIO() {
        if (Robot.isReal()) {
            return new ClimbIOReal();
        } 

        return new ClimbIOSim();
    }
}


