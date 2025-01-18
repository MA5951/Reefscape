
package frc.robot.RobotControl;

import com.ma5951.utils.RobotControl.GenericSuperStracture;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Elevator.Elevator;
import frc.robot.Subsystem.Intake.Intake;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;
import frc.robot.Subsystem.Vision.Vision;

public class SuperStructure extends GenericSuperStracture {

    private static SwerveSubsystem swerve = RobotContainer.swerve;
    private static Vision vision = RobotContainer.vision;
    private static PoseEstimator poseEstimator = RobotContainer.poseEstimator;
    private static Intake intake = RobotContainer.intake;
    private static Arm arm = RobotContainer.arm;
    private static Elevator elevator = RobotContainer.elevator;
    private static Field.ScoringLevel scoringLevel;
    private static Field.ScoringLocation scoringLocation;
    private static Pose2d ejectPose;

    public SuperStructure() {
        super(() -> PoseEstimator.getInstance().getEstimatedRobotPose(),
                () -> SwerveSubsystem.getInstance().getVelocityVector());
        setScoringPreset(Field.ScoringLevel.L1);
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
        return 0d;
    }

    public static boolean isDistanceToIntake() {
        return poseEstimator.getEstimatedRobotPose().getTranslation()
                .getDistance(RobotConstants.ReefCenter) >= RobotConstants.DistanceToBallRemove;
    }

    public static boolean isDistanceToCloseArm() {
        return ejectPose.getTranslation().getDistance(
                poseEstimator.getEstimatedRobotPose().getTranslation()) >= RobotConstants.DistanceToCloseArm;
    }

    public static void updateAngleAdjustController() {
        
    }

    public static void updateXYAdjustController() {

    }

    public static boolean hasGamePiece() {
        return getGamePiece() != Field.GamePiece.NONE;
    }

    public static void update() {

    }

}