
package frc.robot.commands.Elevator;

import com.ma5951.utils.RobotControl.Commands.RobotFunctionStatesCommand;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.RobotContainer;
import frc.robot.RobotControl.SuperStructure;
import frc.robot.Subsystem.Elevator.Elevator;
import frc.robot.Subsystem.Elevator.ElevatorConstants;

public class ElevatorDeafultCommand extends RobotFunctionStatesCommand {
    private static Elevator elevator = RobotContainer.elevator;
    private static Debouncer homeDebouncer = new Debouncer(ElevatorConstants.HOME_TIME);


    public ElevatorDeafultCommand() {
        super(elevator);
        addRequirements(elevator);
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
        elevator.setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void AutomaticLoop() {
        super.AutomaticLoop();
        switch (elevator.getTargetState().getName()) {
            case "IDLE":
                if (elevator.atPoint()) { 
                    elevator.setVoltage(0); 
                } else {
                    elevator.setHight(ElevatorConstants.MIN_HIGHT);
                }
                break;
            case "HOME":
            if (!homeDebouncer.calculate(elevator.getCurrent() < ElevatorConstants.HOME_CURRENT)) {
                elevator.setVoltage(ElevatorConstants.HOME_VOLTAGE);
            } else {
                elevator.resetPose(ElevatorConstants.MIN_HIGHT);
                RobotContainer.setIDLE();
            }
                break;
            case "INTAKE":
                elevator.setHight(ElevatorConstants.HIGHT_INTAKE_CORAL);
                break;
            case "SCORING":
                elevator.setHight(SuperStructure.getScoringPreset().hight);
                break;
            case "BALLREMOVING":
                elevator.setHight(SuperStructure.getBallRemoveHight());
                break;
            case "CLIMB":
                elevator.setVoltage(elevator.getFeedForwardVoltage());
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
        elevator.setVoltage(elevator.getFeedForwardVoltage());
    }

    @Override
    public void ManuelLoop() {
        super.ManuelLoop();
        elevator.setVoltage(-RobotContainer.operatorController.getLeftY() * ElevatorConstants.MANUEL_VOLTAGE_LIMIT);
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