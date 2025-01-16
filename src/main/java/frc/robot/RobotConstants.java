
package frc.robot;

import com.ma5951.utils.RobotControl.StatesTypes.State;
import com.ma5951.utils.Utils.DriverStationUtil;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RobotConstants {

    // Robot Constants
    public static final boolean COMP_LOG = false;
    public static final double kDELTA_TIME = 0.02;

    // States
    public static final State IDLE = new State("IDLE");
    public static final State INTAKE = new State("INTAKE");
    public static final State SCORING = new State("SCORING");
    public static final State BALLREMOVING = new State("BALLREMOVING ");
    public static final State CLIMB = new State("CLIMB ");
    public static final State SORTING = new State("SORTING ");

    
    //Automation Constants
    public static final double DistanceToBallRemove = 2.2;
    public static final double DistanceToCloseArm = 0.3;


    // FieldConstants
    public static final Translation2d FieldZeroCorner = new Translation2d(0, 0);
    public static final Translation2d FieldFarCorner = new Translation2d(17.55, 8.05);// TODO CHECK
    public static final Translation2d FieldMiddlePoint = new Translation2d(17.55 / 2, 8.05 / 2);
    public static final Translation2d ReefCenterBlue = new Translation2d(4.45, 4);// TODO only aproximate
    public static final Translation2d ReefCenterRed = new Translation2d(4.45, 13.2);// TODO only aproximate
    public static final Translation2d ReefCenter = DriverStationUtil.getAlliance() == Alliance.Blue ? ReefCenterBlue
            : ReefCenterRed;

}
