// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.deafultCommands;

import com.ma5951.utils.RobotControl.Commands.RobotFunctionStatesCommand;

import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.Arm.Arm;
import frc.robot.Subsystem.Arm.ArmConstants;
import frc.robot.Subsystem.Intake.Intake;

public class IntakeDeafultCommand extends RobotFunctionStatesCommand {
    private static Intake intake = RobotContainer.intake;

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
                break;
            case "INTAKE":
                break;
            case "SCORING":
                break;
            case "SORTING":
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