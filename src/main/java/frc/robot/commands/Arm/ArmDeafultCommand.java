
package frc.robot.commands.Arm;

import com.ma5951.utils.RobotControl.Commands.RobotFunctionStatesCommand;

import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Arm.ArmConstants;

public class ArmDeafultCommand extends RobotFunctionStatesCommand {
    private static Arm arm = RobotContainer.arm;

    public ArmDeafultCommand() {
        super(arm);
        addRequirements(arm);
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
        arm.setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void AutomaticLoop() {
        super.AutomaticLoop();
        switch (arm.getTargetState().getName()) {
            case "IDLE":
                arm.setVoltage(arm.getFeedForwardVoltage());
                break;
            case "HOLD":
                arm.setAngle(arm.getPosition());
                break;
            case "INTAKE":
                arm.setAngle(ArmConstants.INTAKE_CORALS_ANGLE);
                break;
            case "SCORING":
                arm.setAngle(SuperStructure.getScoringPreset().angle);
                break;
            case "BALLREMOVING":

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
        arm.setVoltage(arm.getFeedForwardVoltage());
    }

    @Override
    public void ManuelLoop() {
        super.ManuelLoop();
        arm.setVoltage(arm.getFeedForwardVoltage());
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