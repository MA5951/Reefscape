// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.deafultCommands;

import com.ma5951.utils.RobotControl.Commands.RobotFunctionStatesCommand;

import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
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
                arm.setAngle(75);
                break;
            case "HOLD":
                arm.setAngle(0);
                break;
            case "INTAKE":
                arm.setAngle(75);
                break;
            case "SCORING":
                arm.setAngle(RobotConstants.SUPER_STRUCTURE.getScoringPreset().angle);
                break;
            case "SORTING":
                arm.setAngle(0);
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