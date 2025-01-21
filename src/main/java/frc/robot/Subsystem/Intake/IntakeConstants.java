
package frc.robot.Subsystem.Intake;


import com.ma5951.utils.RobotControl.StatesTypes.State;
import com.ma5951.utils.RobotControl.StatesTypes.StatesConstants;

import frc.robot.Robot;
import frc.robot.Subsystem.Intake.IOs.IntakeIO;
import frc.robot.Subsystem.Intake.IOs.IntakeIOSim;
import frc.robot.Subsystem.Intake.IOs.IntakeIOReal;

public class IntakeConstants {
    
    public static final double INTAKE_SPEED_BEFORE_FIRST_SENSOR = -4; 
    public static final double INTAKE_FACTOR_AFTER_SENSOR = 1;
    public static final double SORTING_SPEED = -1; 
    public static final double EJECT_SPEED_L1 = 2; 
    public static final double EJECT_SPEED_L234 = 4; 
    public static final double INTAKE_WIDHT = 0; 
    public static final double CORAL_HOLDING_VALUE = 0; 
    //public static final double BALLֹ_EJECTING_VALUE = 0;
    public static final double GEAR = 5;
    public static final int SORTIN_NUM = 3;
    public static final double MANUEL_VOLTAGE_LIMIT = 3;

    
    public static final double PEAK_CURRENT_LIMIT = 35;
    public static final double CONTINUES_CURRENT_LIMIT = 15; 
    public static final double PEAK_CURRENT_TIME = 0.1; 
    public static final boolean IS_CURRENT_LIMIT_ENABLED = true;

    public static final State IDLE = StatesConstants.IDLE;
    public static final State HOLD = new State("HOLD");
    public static final State INTAKE = new State("INTAKE");
    public static final State SCORING = new State("SCORING");
    public static final State SORTING = new State("SORTING");
    public static final State BALLREMOVING  = new State("BALLREMOVING ");

    public static final State[] SUBSYSTEM_STATES = new State[] {IDLE , HOLD , INTAKE , SCORING , SORTING , BALLREMOVING};

    public static final IntakeIO getIntakeIO() {
        if (Robot.isReal()) {
            return new IntakeIOReal();
        } 
        return new IntakeIOSim();
    }
}
