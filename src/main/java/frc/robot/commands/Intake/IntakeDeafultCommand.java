
package frc.robot.commands.Intake;

import com.ma5951.utils.RobotControl.Commands.RobotFunctionStatesCommand;

import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystem.Arm.ArmConstants;
import frc.robot.Subsystem.Intake.Intake;
import frc.robot.Subsystem.Intake.IntakeConstants;

public class IntakeDeafultCommand extends RobotFunctionStatesCommand {
    private static Intake intake = RobotContainer.intake;
    private static int sortingNum = 0;
    private static Boolean updatedSortin = false;
    private static boolean hasEntered = false;

    public IntakeDeafultCommand() {
        super(intake);
        addRequirements(intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void AutomaticLoop() {
        super.AutomaticLoop();
        switch (intake.getTargetState().getName()) {
            case "IDLE":
                intake.setVoltage(0);
                hasEntered = false;
                break;
            case "HOLD":
                intake.setVoltage(SuperStructure.getCoralHoldValue());

                if (RobotContainer.currentRobotState == RobotConstants.SCORING && RobotContainer.arm.atPoint()) {
                    intake.setTargetState(IntakeConstants.SCORING);
                }
                break;
            case "INTAKE":
                if (!intake.getRearSensor() && !hasEntered) {
                    intake.setVoltage(IntakeConstants.INTAKE_SPEED_BEFORE_FIRST_SENSOR);
                    SuperStructure.updatePose();

                } else if (intake.getRearSensor() && intake.getFrontSensor()) {
                    intake.setVoltage(IntakeConstants.SORTING_SPEED);
                    hasEntered = true;
                } else {
                    intake.setTargetState(IntakeConstants.IDLE);
                }
                break;
            case "SCORING":
                intake.setVoltage(SuperStructure.getScoringPreset().ejectVolt);
                SuperStructure.updatePose();
                break;
            case "SORTING":
                if (sortingNum > IntakeConstants.SORTIN_NUM) {
                    RobotContainer.setIDLE();
                    RobotContainer.arm.setTargetState(ArmConstants.HOLD);
                    intake.setTargetState(IntakeConstants.HOLD);
                    sortingNum = 0;
                    updatedSortin = false;
                }

                if (intake.getRearSensor()) {
                    intake.setVoltage(IntakeConstants.SORTING_SPEED);
                    if (!updatedSortin) {
                        sortingNum++;
                        updatedSortin = true;
                    }
                } else if (!intake.getRearSensor()) {
                    updatedSortin = false;
                    intake.setVoltage(-IntakeConstants.SORTING_SPEED);
                }

                break;
            case "BALLREMOVING":
                intake.setVoltage(IntakeConstants.BALLREMOVING_VOLTAGE);
                break;
            case "SKYHOOK":
                intake.setVoltage(-0.5);
                break;
            case "EJECT":
                if (SuperStructure.isIntakeFliped()) {
                    intake.setVoltage(6);
                } else {
                    intake.setVoltage(-6);
                }
        }
    }

    @Override
    public void CAN_MOVE() {
        super.CAN_MOVE();
    }

    @Override
    public void CANT_MOVE() {
        super.CANT_MOVE();
        intake.setVoltage(0);
        hasEntered = false;
    }

    @Override
    public void ManuelLoop() {
        super.ManuelLoop();
        // if (RobotContainer.operatorController.getL1Button()) {
        //     intake.setVoltage(-IntakeConstants.MANUEL_VOLTAGE_LIMIT);
        // } else if (RobotContainer.operatorController.getR1Button()) {
        //     intake.setVoltage(IntakeConstants.MANUEL_VOLTAGE_LIMIT);
        // } else {
        //     intake.setVoltage(0);
        // }
    }

    @Override
    public void AutoLoop() {
        super.AutoLoop();
        AutomaticLoop();
    }

    @Override
    public void TestLoop() {
        super.TestLoop();
    }
}