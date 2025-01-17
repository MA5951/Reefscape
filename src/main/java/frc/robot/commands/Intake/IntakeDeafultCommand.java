
package frc.robot.commands.Intake;

import com.ma5951.utils.RobotControl.Commands.RobotFunctionStatesCommand;

import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
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
                break;
            case "INTAKE":
                intake.setVoltage(IntakeConstants.INTAKE_SPEED_BEFORE_FIRST_SENSOR);
                break;
            case "SCORING":
                intake.setVoltage(SuperStructure.getScoringPreset().ejectVolt);
                break;
            case "SORTING":
                if (sortingNum == 5) {
                    sortingNum = 0;
                }

                if (sortingNum > 4) {
                    RobotContainer.setIDLE();
                } else {
                    
                    if (intake.getRearSensor()) {
                        updatedSortin = false;
                    }

                    if (!updatedSortin && !intake.getRearSensor()) {
                        updatedSortin = true;
                        sortingNum++;
                    }
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