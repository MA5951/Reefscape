
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
                break;
            case "HOLD":
                intake.setVoltage(0);

                if (RobotContainer.arm.atPoint() && RobotContainer.currentRobotState == RobotConstants.SCORING) {
                    intake.setTargetState(IntakeConstants.SCORING);
                }

                
                break;
            case "INTAKE":
                intake.setVoltage(IntakeConstants.INTAKE_SPEED_BEFORE_FIRST_SENSOR);
                break;
            case "SCORING":
                intake.setVoltage(SuperStructure.getScoringPreset().ejectVolt);
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
                    intake.setVoltage(0);
                    if (!updatedSortin) {
                        sortingNum++;
                        updatedSortin = true;
                    }
                } else if (!intake.getRearSensor()) {
                    updatedSortin = false;
                    intake.setVoltage(0);
                }
                

                break;
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
    }

    @Override
    public void ManuelLoop() {
        super.ManuelLoop();
        intake.setVoltage(0);
    }

    @Override
    public void AutoLoop() {
        super.AutoLoop();
    }

    @Override
    public void TestLoop() {
        super.TestLoop();
    }
}