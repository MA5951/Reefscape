
package frc.robot.Subsystem.Intake;

import frc.robot.Robot;
import frc.robot.Subsystem.Intake.IOs.IntakeIO;
import frc.robot.Subsystem.Intake.IOs.IntakeIOSim;
import frc.robot.Subsystem.Intake.IOs.IntakeIOreal;

public class IntakeConstants {
    public static final double INTAKE_SPEED_BEFORE_FIRST_SENSOR = 0; 
    public static final double FACTOR_AFTER_SENSOR = 1;
    public static final double SORTING_SPEED = 0; 
    public static final double EJECT_SPEED_L1 = 0; 
    public static final double EJECT_SPEED_L234 = 0; 
    public static final double INTAKE_WIDHT = 0; 
    public static final double CORAL_HOLDING_VALUE = 0; 
    public static final double BALL_SPEED = 0; 
    public static final double BALL_HOLDING_VALUE = 0;
    public static final double GEAR = 1;

    public static final double PEAK_CURRENT_LIMIT = 35;
    public static final double CONTINUES_CURRENT_LIMIT = 20; 
    public static final double PEAK_CURRENT_TIME = 0.1; 
    public static final boolean IS_CURRENT_LIMIT_ENABLED = true;

    public static final IntakeIO getIntakeIO() {
        if (Robot.isReal()) {
            return new IntakeIOreal();
        } 
        return new IntakeIOSim();
    }
}
