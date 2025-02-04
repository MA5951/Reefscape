
package frc.robot;

import com.ma5951.utils.RobotControl.StatesTypes.State;
import com.ma5951.utils.Utils.DriverStationUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RobotConstants {

    // Robot Constants
    public static final boolean COMP_LOG = false;
    public static final boolean USE_LEDS = true;
    public static final double kDELTA_TIME = 0.02;
    public static final double NOMINAL_VOLTAGE = 12d;
    public static final Pose3d SIM_ARM_OFFSET = new Pose3d(0.035, -0.002 ,  0.613, new Rotation3d(0, 0, 0));

    // States
    public static final State IDLE = new State("IDLE");
    public static final State INTAKE = new State("INTAKE");
    public static final State SCORING = new State("SCORING");
    public static final State BALLREMOVING = new State("BALLREMOVING ");
    public static final State CLIMB = new State("CLIMB ");
    public static final State SORTING = new State("SORTING ");
    public static final State EJECT = new State("EJECT");
    public static final State SKYHOOK = new State("SKYHOOK");
    public static final State HOLDBALL = new State("HOLDBALL");

    // Automation Constants
    public static final double DistanceToBallRemove = 2.2;
    public static final double DistanceToCloseArm = 0.1;
    public static final double DistanceOffsetScoring = 0.1;
    public static final double DistanceToAlign = 1;
    public static final double DistanceToRelativAlign = 0.6;
    public static final double SpeedToScore = 0.5;
    public static final double ToloranceToScoring = 0.02;
    public static final double DistanceToDriveBalls = 0.7;

    // FieldConstants
    public static final Translation2d FieldZeroCorner = new Translation2d(0, 0);
    public static final Translation2d FieldFarCorner = new Translation2d(17.548, 8.052);
    public static final Translation2d FieldMiddlePoint = new Translation2d(17.548 / 2, 8.052 / 2);
    public static final Translation2d ReefCenterBlue = new Translation2d(4.45, 4);//TODO
    public static final Translation2d ReefCenterRed = new Translation2d(4.45, 13.2);//TODO
    public static final Translation2d ReefCenter = DriverStationUtil.getAlliance() == Alliance.Blue ? ReefCenterBlue
            : ReefCenterRed;

    //Reef
    public static final Pose2d Tag6Pose = new Pose2d(13.474446, 3.3063179999999996, Rotation2d.fromDegrees(120));
    public static final Pose2d Tag7Pose = new Pose2d(13.890498, 4.0259, Rotation2d.fromDegrees(180));
    public static final Pose2d Tag8Pose = new Pose2d(13.474446, 4.745482, Rotation2d.fromDegrees(-120));
    public static final Pose2d Tag9Pose = new Pose2d(12.643358, 4.745482, Rotation2d.fromDegrees(-60));
    public static final Pose2d Tag10Pose = new Pose2d(12.227305999999999, 4.0259, new Rotation2d(0));
    public static final Pose2d Tag11Pose = new Pose2d(12.643358, 3.3063179999999996, Rotation2d.fromDegrees(60));

    public static final Pose2d Tag17Pose = new Pose2d(4.073905999999999, 3.3063179999999996, Rotation2d.fromDegrees(60));
    public static final Pose2d Tag18Pose = new Pose2d(3.6576, 4.0259, new Rotation2d(0));
    public static final Pose2d Tag19Pose = new Pose2d(4.073905999999999, 4.745482, Rotation2d.fromDegrees(-60));
    public static final Pose2d Tag20Pose = new Pose2d(4.904739999999999, 4.745482, Rotation2d.fromDegrees(-120));
    public static final Pose2d Tag21Pose = new Pose2d(5.321046, 4.0259, Rotation2d.fromDegrees(180));
    public static final Pose2d Tag22Pose = new Pose2d(4.904739999999999, 3.3063179999999996, Rotation2d.fromDegrees(120));


    //Source
    public static final Pose2d Tag1Pose = new Pose2d(16.697198, 0.65532, Rotation2d.fromDegrees(-52));
    public static final Pose2d Tag2Pose = new Pose2d(16.697198, 7.3964799999999995, Rotation2d.fromDegrees(52));
    public static final Pose2d Tag12Pose = new Pose2d(0.851154, 0.65532, Rotation2d.fromDegrees(-127));
    public static final Pose2d Tag13Pose = new Pose2d(0.851154, 7.3964799999999995, Rotation2d.fromDegrees(127));

}
