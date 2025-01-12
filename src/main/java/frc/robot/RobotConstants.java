
package frc.robot;



import com.ma5951.utils.RobotControl.StatesTypes.State;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotControl.SuperStructure;

public class RobotConstants {

    //Robot Constants
    public static final boolean COMP_LOG = false;
    public static final double kDELTA_TIME = 0.02;

    //Robot Control
    public static final SuperStructure SUPER_STRUCTURE = new SuperStructure();
    
    //States
    public static final State IDLE = new State("IDLE");
    public static final State INTAKE = new State("IDLE");
    public static final State SCORING = new State("SCORING");

    //FieldConstants
    public static final Translation2d FieldZeroCorner = new Translation2d(0 , 0);
    public static final Translation2d FieldFarCorner = new Translation2d(17.55 , 8.05);
    public static final Translation2d FieldMiddlePoint = new Translation2d(17.55 / 2, 8.05 / 2);


}
