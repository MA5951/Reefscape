
package frc.robot.Subsystem.climb;

import com.ma5951.utils.RobotControl.StatesTypes.State;
import com.ma5951.utils.RobotControl.StatesTypes.StatesConstants;

import frc.robot.Robot;
import frc.robot.Subsystem.climb.IOs.ClimbIO;
import frc.robot.Subsystem.climb.IOs.ClimbIOReal;
import frc.robot.Subsystem.climb.IOs.ClimbIOSim;

public class climbConstants {
    public static final double kCURRENT_LIMIT = 30;
    public static final double kCONTINUOUS_LOWER_LIMIT = 25;
    public static final double kCONTINUOUS_CURRENT_TIME = 0.1;
    public static final Boolean kENABLE_CURRENT_LIMIT = true;
    public static final double GEAR = 1;

    public static final double ROBOT_WIGHT = 0;

    public static final State IDLE = StatesConstants.IDLE;

    public static final State[] SUBSYSTEM_STATES = new State[] {IDLE };



    public static ClimbIO getClimbIO() {
        if (Robot.isReal()) {
            return new ClimbIOReal();
        } else {
            return new ClimbIOSim();
        }
    }
}


