
package frc.robot.RobotControl;

import com.ma5951.utils.RobotControl.GenericSuperStracture;
import com.ma5951.utils.RobotControl.StatesTypes.State;
import com.ma5951.utils.Utils.DriverStationUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Elevator.Elevator;
import frc.robot.Subsystem.Elevator.ElevatorConstants;
import frc.robot.Subsystem.Intake.Intake;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveConstants;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;
import frc.robot.Subsystem.Vision.Vision;
import frc.robot.Subsystem.Vision.VisionConstants;
import frc.robot.Utils.ReefFace;
import frc.robot.commands.Swerve.TeleopSwerveController;

@SuppressWarnings("unused")
public class SuperStructure extends GenericSuperStracture {

    private static SwerveSubsystem swerve = RobotContainer.swerve;
    private static Vision vision = RobotContainer.vision;
    private static PoseEstimator poseEstimator = RobotContainer.poseEstimator;
    private static Intake intake = RobotContainer.intake;
    private static Arm arm = RobotContainer.arm;
    private static Elevator elevator = RobotContainer.elevator;
    private static Field.ScoringLevel scoringLevel;
    private static Field.ScoringLocation scoringLocation;
    private static ReefFace scoringFace;
    private static Pose2d ejectPose;

    public SuperStructure() {
        super(() -> PoseEstimator.getInstance().getEstimatedRobotPose(),
                () -> SwerveSubsystem.getInstance().getVelocityVector());
        setScoringPreset(Field.ScoringLevel.L1);
        setScoringLocation(Field.ScoringLocation.RIGHT);
    }

    public static void setScoringPreset(Field.ScoringLevel ScoringLevel) {
        scoringLevel = ScoringLevel;
    }

    public static Field.ScoringLevel getScoringPreset() {
        return scoringLevel;
    }

    public static void setScoringLocation(Field.ScoringLocation ScoringLocation) {
        scoringLocation = ScoringLocation;
    }

    public static double getBallRemoveHight() {
        if (Field.isReefTag(RobotContainer.vision.getTagID())) {
            return Field.getFaceByID(RobotContainer.vision.getTagID()).BallHight() == Field.BallHight.HIGH ? ElevatorConstants.HIGHT_EJECT_BALL_HIGH :
            ElevatorConstants.HIGHT_EJECT_BALL_LOW;
        } else {
            return Field.getClosestReefFace(currentPoseSupplier.get()).BallHight() == Field.BallHight.HIGH ? ElevatorConstants.HIGHT_EJECT_BALL_HIGH :
            ElevatorConstants.HIGHT_EJECT_BALL_LOW;
        }
    }

    public static Field.GamePiece getGamePiece() {
        if (intake.getFrontSensor()) {
            return Field.GamePiece.CORAL;
        }
        return Field.GamePiece.NONE;
    }

    public static boolean isIntakeFliped() {
        return intake.getPosition() > 90;
    }

    public static double getElevatorHight() {
        return elevator.getHight();
    }

    public static double getCoralHoldValue() {
        return 0d; //TODO write the code...
    }

    public static boolean isDistanceToIntake() {
        return poseEstimator.getEstimatedRobotPose().getTranslation()
                .getDistance(RobotConstants.ReefCenter) >= RobotConstants.DistanceToBallRemove; //TODO chack if need abs in the red/blue side why ball remove?
                //TODO i will chek if you in the src zone
    }

    public static boolean isDistanceToCloseArm() {
        return ejectPose.getTranslation().getDistance(
                poseEstimator.getEstimatedRobotPose().getTranslation()) >= RobotConstants.DistanceToCloseArm; //TODO chack if need abs in the red/blue side 
    } 

    public static double getAngleForIntakeAlign() {
        if (RobotContainer.alliance == Alliance.Red) {
            return currentPoseSupplier.get().getY() > RobotConstants.FieldMiddlePoint.getY()
                    ? RobotConstants.Tag13Pose.getRotation().getDegrees()
                    : RobotConstants.Tag12Pose.getRotation().getDegrees();

        } else {
            return currentPoseSupplier.get().getY() > RobotConstants.FieldMiddlePoint.getY()
                    ? RobotConstants.Tag2Pose.getRotation().getDegrees()
                    : RobotConstants.Tag1Pose.getRotation().getDegrees();
        }
    }

    public static void updateScoringFace() {
        scoringFace = Field.getClosestReefFace(currentPoseSupplier.get()); 
    }

    public static void updateAngleAdjustController(State robotState) { //TODO use the currentRobotState static vairable 
        if (robotState == RobotConstants.INTAKE) {
            TeleopSwerveController.angleAdjustController.setSetPoint(getAngleForIntakeAlign());
        } else if (robotState == RobotConstants.SCORING) {
            TeleopSwerveController.angleAdjustController.setSetPoint(scoringFace.AbsAngle());
        }
        //TODO add a case for climbe?
    }

    public static void updateXYAdjustController(State robotState) {
        if (robotState == RobotConstants.SCORING && RobotContainer.vision.getTagID() == scoringFace.TagID() &&
                currentPoseSupplier.get().getTranslation()
                        .getDistance(scoringFace.tagPose().getTranslation()) < RobotConstants.DistanceToRelativAlign) {
            TeleopSwerveController.autoAdjustXYController.updateSetPoint(VisionConstants.RELATIV_REEF_SET_POINT);
            TeleopSwerveController.autoAdjustXYController.updateMeaurment(() -> RobotContainer.vision.getPoseForRelativReefAlign());
            TeleopSwerveController.autoAdjustXYController.setPID(
                SwerveConstants.REL_X_KP, 
                SwerveConstants.REL_X_KI, 
                SwerveConstants.REL_X_KD, 
                SwerveConstants.REL_XY_TOLORANCE, 
                SwerveConstants.REL_Y_KP, 
                SwerveConstants.REL_Y_KI, 
                SwerveConstants.REL_Y_KD, 
                SwerveConstants.REL_XY_TOLORANCE);
            TeleopSwerveController.autoAdjustXYController.setConstrains(SwerveConstants.REL_XY_CONSTRAINTS);
            TeleopSwerveController.autoAdjustXYController.setField(false);
        }else if (robotState == RobotConstants.SCORING) { //TODO move the scoring state up and write if else 
            TeleopSwerveController.autoAdjustXYController.updateSetPoint(scoringFace.getAlignPose());
            TeleopSwerveController.autoAdjustXYController.updateMeaurment(currentPoseSupplier);
            TeleopSwerveController.autoAdjustXYController.setPID(
                SwerveConstants.ABS_X_KP, 
                SwerveConstants.ABS_X_KI, 
                SwerveConstants.ABS_X_KD, 
                SwerveConstants.ABS_XY_TOLORANCE, 
                SwerveConstants.ABS_Y_KP, 
                SwerveConstants.ABS_Y_KI, 
                SwerveConstants.ABS_Y_KD, 
                SwerveConstants.ABS_XY_TOLORANCE);
            TeleopSwerveController.autoAdjustXYController.setConstrains(SwerveConstants.ABS_XY_CONSTRAINTS);
            TeleopSwerveController.autoAdjustXYController.setField(true);
        }
    }

    public static boolean hasGamePiece() {
        return getGamePiece() != Field.GamePiece.NONE;
    }

    public static void update() {
        //log the func?
    }

}