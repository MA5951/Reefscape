
package frc.robot.RobotControl;


import com.ma5951.utils.RobotControl.GenericSuperStracture;

import frc.robot.RobotContainer;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Intake.Intake;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;
import frc.robot.Subsystem.Vision.Vision;

public class SuperStructure extends GenericSuperStracture {

    public static SwerveSubsystem swerve =  RobotContainer.swerve;
    public static Vision vision =  RobotContainer.vision;
    public static PoseEstimator poseEstimator =  RobotContainer.poseEstimator;
    public static Intake intake =  Intake.getInstance();
    public static Arm arm =  Arm.getInstance();
    public static Field.ScoringLevel scoringLevel;

    private double lastIntakeHight = 0;

    public SuperStructure() {
        super(() -> PoseEstimator.getInstance().getEstimatedRobotPose(),
                () -> SwerveSubsystem.getInstance().getVelocityVector());
    }

    public void setScoringPreset(Field.ScoringLevel ScoringLevel) {
        scoringLevel = ScoringLevel;
    }

    public Field.ScoringLevel getScoringPreset() {
        return scoringLevel;
    }

    public Field.GamePiece getGamePiece() {
        if (intake.getFrontSensor() && lastIntakeHight == 0) { //TODO
            return Field.GamePiece.CORAL;
        } else if (intake.getFrontSensor() && lastIntakeHight == 1) { //TODO
            return Field.GamePiece.BALL;
        }

        return Field.GamePiece.NONE;
    }

    @Override
    public boolean hasGamePiece() {
        return getGamePiece() != Field.GamePiece.NONE;
    }

    public void setIntakeHight(double intakeHight) {
        lastIntakeHight = intakeHight;
    }

    public void update() {

    }

}
