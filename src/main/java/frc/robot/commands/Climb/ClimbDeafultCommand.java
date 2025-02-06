
package frc.robot.commands.Climb;

import com.ma5951.utils.RobotControl.Commands.RobotFunctionStatesCommand;
import com.ma5951.utils.Utils.BooleanLatch;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.Arm.ArmConstants;
import frc.robot.Subsystem.Climb.Climb;
import frc.robot.Subsystem.Climb.ClimbConstants;

public class ClimbDeafultCommand extends RobotFunctionStatesCommand {
    private static Climb climb = RobotContainer.climb;
    private Debouncer servoDebounder;
    private boolean openServo = false;
    private BooleanLatch wasAtClimb;

    public ClimbDeafultCommand() {
        super(climb);
        servoDebounder = new Debouncer(10);
        wasAtClimb = new BooleanLatch(() -> climb.atClimbAngle());
        wasAtClimb.reset();
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.setServo(ClimbConstants.FREE_POSITION);
        wasAtClimb.reset();
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
                climb.setServo(ClimbConstants.FREE_POSITION);
                climb.setVoltage(0);
                break;
            case "OPEN_RACHET":
                // climb.setServo(ClimbConstants.FREE_POSITION);
                // climb.setVoltage(0.32);
                // openServo = true;

                if (true) { // servoDebounder.calculate(openServo)
                    climb.setTargetState(ClimbConstants.ALIGN);
                }
            case "ALIGN":

                climb.setVoltage(ClimbConstants.ALIGN_VOLTAGE);

                if (RobotContainer.driverController.getSquareButton() && climb.atAlignAngle()) {
                    climb.setTargetState(ClimbConstants.CLIMB);
                }
                break;
            case "CLIMB":
                climb.setServo(ClimbConstants.LOCK_POSITION);

                if (!wasAtClimb.get()) {
                    climb.setVoltage(ClimbConstants.CLIMB_VOLTAGE);
                }

                // if (RobotContainer.driverController.getSquareButton() &&
                // !climb.atAlignAngle()) {
                // climb.setTargetState(ClimbConstants.);
                // }
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
        // if (RobotContainer.driverController.getSquareButton() &&
        // climb.getTargetState() == ClimbConstants.ALIGN) {
        // climb.setTargetState(ClimbConstants.CLIMB);
        // }
        climb.setVoltage(0);

    }

    @Override
    public void ManuelLoop() {
        super.ManuelLoop();
        climb.setVoltage(-RobotContainer.driverController.getRightY() * ArmConstants.kMANUEL_VOLTAGE_LIMIT);

        if (RobotContainer.driverController.getPOV() == 270) {
            climb.setServo(ClimbConstants.LOCK_POSITION);
        } else if (RobotContainer.driverController.getPOV() == 90) {
            climb.setServo(ClimbConstants.FREE_POSITION);

        }

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