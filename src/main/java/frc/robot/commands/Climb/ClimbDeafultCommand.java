
package frc.robot.commands.Climb;

import com.ma5951.utils.RobotControl.Commands.RobotFunctionStatesCommand;

import frc.robot.RobotContainer;
import frc.robot.Subsystem.Arm.ArmConstants;
import frc.robot.Subsystem.Climb.Climb;
import frc.robot.Subsystem.Climb.ClimbConstants;

public class ClimbDeafultCommand extends RobotFunctionStatesCommand {
    private static Climb climb = RobotContainer.climb;

    public ClimbDeafultCommand() {
        super(climb);
        addRequirements(climb);
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
        climb.setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void AutomaticLoop() {
        super.AutomaticLoop();
        switch (climb.getTargetState().getName()) {
            case "IDLE":
                if (climb.getPosition() > ClimbConstants.ALIGN_ANGLE) {
                    climb.setVoltage(ClimbConstants.IDLE_VOLTAGE);
                } else {
                    climb.setVoltage(0);
                }
                break;
            case "ALIGN":
                climb.setVoltage(ClimbConstants.ALIGN_VOLTAGE);

                if (RobotContainer.driverController.getSquareButton() && climb.atAlignAngle()) {
                    climb.setTargetState(ClimbConstants.CLIMB);
                }
                break;
            case "CLIMB":
                climb.setVoltage(ClimbConstants.CLIMB_VOLTAGE);

                if (RobotContainer.driverController.getSquareButton() && !climb.atAlignAngle()) {
                    climb.setTargetState(ClimbConstants.ALIGN);
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
        climb.setVoltage(0);
    }

    @Override
    public void ManuelLoop() {
        super.ManuelLoop();
        climb.setVoltage(-RobotContainer.driverController.getRightY() * ArmConstants.kMANUEL_VOLTAGE_LIMIT);
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