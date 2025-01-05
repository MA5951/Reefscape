
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

    //FieldConstants
    public static final Translation2d FieldZeroCorner = new Translation2d(0 , 0);
    public static final Translation2d FieldFarCorner = new Translation2d(16.58 , 8.20);
    public static final Translation2d FieldMiddlePoint = new Translation2d(16.58 / 2, 8.20 / 2);


}
