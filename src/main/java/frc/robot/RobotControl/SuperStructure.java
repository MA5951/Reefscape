
package frc.robot.RobotControl;

import java.util.function.Supplier;

import org.photonvision.simulation.VideoSimUtil;

import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Logger.LoggedPose2d;
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
import frc.robot.Utils.ModuleLimits;
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
    public static boolean isScoringAutomatic = true;
    private static LoggedPose2d reefFace;
    private static LoggedDouble alignAngle;
    public static boolean isFine;
    public static boolean isFinalLeft;
    public static boolean isFinalRight;

    public SuperStructure() {
        super(() -> PoseEstimator.getInstance().getEstimatedRobotPose(),
                () -> SwerveSubsystem.getInstance().getVelocityVector());
        setScoringPreset(Field.ScoringLevel.L4);
        setScoringLocation(Field.ScoringLocation.RIGHT);

        reefFace = new LoggedPose2d("/SuperStructure/Reef Face");
        alignAngle = new LoggedDouble("/SuperStructure/Align Angle");
        ejectPose = new Pose2d();
        updateScoringFace();

        swerve.setCurrentLimits(updateSwerveLimits());
    }

    public static void toggleAutoScoring() {
        if (isScoringAutomatic) {
            isScoringAutomatic = false;
        } else {
            isScoringAutomatic = true;
        }
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
            return Field.getFaceByID(RobotContainer.vision.getTagID()).BallHight().elevatorHight;
        } else {
            return Field.getClosestReefFace(currentPoseSupplier.get()).BallHight().elevatorHight;
        }
    }

    public static Field.GamePiece getGamePiece() {
        if (intake.getFrontSensor() || intake.getRearSensor()) {
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
        return RobotContainer.arm.getVelocity() / 107 * 1.5;
    }

    public static void updatePose() {
        ejectPose = currentPoseSupplier.get();
    }

    public static boolean isDistanceToCloseArm() {
        return RobotContainer.currentRobotState == RobotConstants.SCORING ? ejectPose.getTranslation().getDistance(
                currentPoseSupplier.get().getTranslation()) >= RobotConstants.DistanceToCloseArm
                        + RobotConstants.DistanceOffsetScoring
                : ejectPose.getTranslation().getDistance(
                        currentPoseSupplier.get().getTranslation()) >= RobotConstants.DistanceToCloseArm;
    }

    public static boolean isDistanceToEndBallRemove() {
        return ejectPose.getTranslation().getDistance(
                currentPoseSupplier.get().getTranslation()) >= RobotConstants.DistanceToDriveBalls;
    }

    public static double getAngleForIntakeAlign() {
        if (RobotContainer.alliance == Alliance.Red) {
            return currentPoseSupplier.get().getY() > RobotConstants.FieldMiddlePoint.getY()
                    ? RobotConstants.Tag1Pose.getRotation().getDegrees()
                    : RobotConstants.Tag2Pose.getRotation().getDegrees();

        } else {
            return currentPoseSupplier.get().getY() > RobotConstants.FieldMiddlePoint.getY()
                    ? RobotConstants.Tag13Pose.getRotation().getDegrees()
                    : RobotConstants.Tag12Pose.getRotation().getDegrees();
        }
    }

    public static void updateScoringFace() {
        scoringFace = Field.getClosestReefFace(currentPoseSupplier.get());
    }

    public static void updateAngleAdjustController() {
        if (RobotContainer.currentRobotState == RobotConstants.INTAKE) {
            TeleopSwerveController.angleAdjustController.setSetPoint(getAngleForIntakeAlign());
        } else if (RobotContainer.currentRobotState == RobotConstants.SCORING || RobotContainer.currentRobotState == RobotConstants.BALLREMOVING) {
            TeleopSwerveController.angleAdjustController.setSetPoint(scoringFace.AbsAngle());
        }
    }

    public static void setAbsXY() {
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
    }

    public static void setAbsXYBalls() {
        TeleopSwerveController.autoAdjustXYController.updateSetPoint(scoringFace.getAlignPose());
        TeleopSwerveController.autoAdjustXYController.updateMeaurment(currentPoseSupplier);
        TeleopSwerveController.autoAdjustXYController.setPID(
                SwerveConstants.ABS_X_KP /1.5,
                SwerveConstants.ABS_X_KI,
                SwerveConstants.ABS_X_KD,
                SwerveConstants.ABS_XY_TOLORANCE,
                SwerveConstants.ABS_Y_KP / 1.5,
                SwerveConstants.ABS_Y_KI,
                SwerveConstants.ABS_Y_KD,
                SwerveConstants.ABS_XY_TOLORANCE);
    }

    public static String updateXYAdjustController() {
        if (RobotContainer.currentRobotState == RobotConstants.SCORING) {
            if ((isDitancetToFineAlign() || isFine) && arm.atPoint() && elevator.atPoint()) {
                if (scoringLocation == Field.ScoringLocation.LEFT) {
                    TeleopSwerveController.reefXController.updateSetPoint(VisionConstants.RELATIV_LEFT_REEF_SET_POINT);
                    if (isDitancetToFinalAlignLeft() || isFinalLeft) {
                        TeleopSwerveController.autoAdjustXYController
                                .updateSetPoint(scoringFace.getLeftAlignPose());
                        isFinalLeft = true;
                    } else {
                        TeleopSwerveController.autoAdjustXYController
                                .updateSetPoint(scoringFace.getLeftSemiAlignPose());
                    }
                } else if ((scoringLocation == Field.ScoringLocation.RIGHT || isFinalRight) && arm.atPoint()
                        && elevator.atPoint()) {
                    if (isDitancetToFinalAlignRight()) {
                        TeleopSwerveController.reefXController.updateSetPoint(VisionConstants.RELATIV_RIGHT_REEF_SET_POINT);
                        TeleopSwerveController.autoAdjustXYController
                                .updateSetPoint(scoringFace.getRightAlignPose());
                        isFinalRight = true;
                    } else {
                        TeleopSwerveController.autoAdjustXYController
                                .updateSetPoint(scoringFace.getRightSemiAlignPose());
                    }
                }

                isFine = true;

                return "Scoring ABS XY Final Pose";
            } else if (!isFine && !isFinalLeft && !isFinalRight) {
                TeleopSwerveController.autoAdjustXYController.updateSetPoint(scoringFace.getAlignPose());
                return "Scoring ABS XY";
            }
            return "NONE";
        } else if (RobotContainer.currentRobotState == RobotConstants.BALLREMOVING) {
            
            if (isDitancetToFineAlign() || isFine) {
                TeleopSwerveController.autoAdjustXYController
                        .updateSetPoint(scoringFace.getBallRemovingPose());
                isFine = true;
                return "BALLS ABS XY Final Pose";
            } else {
                TeleopSwerveController.autoAdjustXYController
                        .updateSetPoint(scoringFace.getAlignPose());
                return "BALLS ABS XY Pose";
            }
        } else {
            return "NONE";
        }

    }

    public static boolean isDitancetToFineAlign() {
        return currentPoseSupplier.get().getTranslation()
                .getDistance(
                        scoringFace.getAlignPose().getTranslation()) < RobotConstants.DistanceToRelativAlign;
    }

    public static boolean isDitancetToFinalAlignLeft() {
        return currentPoseSupplier.get().getTranslation()
                .getDistance(
                        scoringFace.getLeftSemiAlignPose().getTranslation()) < 0.1;
    }

    public static boolean isDitancetToFinalAlignRight() {
        return currentPoseSupplier.get().getTranslation()
                .getDistance(
                        scoringFace.getRightSemiAlignPose().getTranslation()) < 0.1;
    }

    public static boolean isDitancetToOpenSystems() {
        return currentPoseSupplier.get().getTranslation()
                .getDistance(
                        scoringFace.tagPose().getTranslation()) < 2.5;
    }

    public static boolean isDitancetToScore() {
        return currentPoseSupplier.get().getTranslation()
                .getDistance(
                        scoringFace.getLeftAlignPose().getTranslation()) < 0.07
                || currentPoseSupplier.get().getTranslation()
                        .getDistance(
                                scoringFace.getRightAlignPose().getTranslation()) < 0.07;
    }

    public static boolean isDitancetToBallRemove() {
        return currentPoseSupplier.get().getTranslation()
                .getDistance(
                        scoringFace.getBallRemovingPose().getTranslation()) < 0.07;
                
    }

    public static boolean hasGamePiece() {
        return getGamePiece() != Field.GamePiece.NONE;
    }

    public static void update() {
        reefFace.update(scoringFace.getAlignPose());
        alignAngle.update(scoringFace.AbsAngle());
    }

    public static Supplier<ModuleLimits> updateSwerveLimits() {
        if (elevator.getHight() > SwerveConstants.HIGHT_TO_LIMIT) {
            return () -> SwerveConstants.OPEN_ELEVATOR;
        } 
        
        return () -> SwerveConstants.DEFUALT;
    }

}