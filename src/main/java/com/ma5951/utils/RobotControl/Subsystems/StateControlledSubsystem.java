// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.RobotControl.Subsystems;

import java.util.Arrays;
import java.util.List;

import com.ma5951.utils.DashBoard.MAShuffleboard;
import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedString;
import com.ma5951.utils.RobotControl.StatesTypes.State;
import com.ma5951.utils.RobotControl.StatesTypes.StatesConstants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public abstract class StateControlledSubsystem extends SubsystemBase {

    private List<State> systemStatesList;
    private boolean systemCanMove = true;
    private State systemFunctionState = StatesConstants.AUTOMATIC;
    private State targetState;
    private State lastState;
    private LoggedString currentStateLog;
    private LoggedString systemFunctionStateLog;
    private LoggedString targetStateLog;
    private LoggedBool systemCanMoveLog;
    private String systemName;
    protected MAShuffleboard board;

    public StateControlledSubsystem(State[] states, String name) {
        targetState = StatesConstants.IDLE;
        systemStatesList = Arrays.asList(states);
        currentStateLog = new LoggedString("/RobotControl/" + name + "/Current State");
        systemFunctionStateLog = new LoggedString("/RobotControl/" + name + "/System Function State");
        targetStateLog = new LoggedString("/RobotControl/" + name + "/Target State");
        systemCanMoveLog = new LoggedBool("/RobotControl/" + name + "/Can Move");
        board = new MAShuffleboard(name);
        systemName = name;
        System.out.println(systemName + " Manuel");
        board.addBoolean(systemName + " Manuel", false);

    }

    public void setSystemFunctionState(State FunctioState) {
        if (FunctioState == StatesConstants.MANUEL) {
            board.addBoolean(systemName + " Manuel", true);
        } else {
            board.addBoolean(systemName + " Manuel", false);
        }

        systemFunctionState = FunctioState;
    }

    public State getSystemFunctionState() {
        return systemFunctionState;
    }

    // add get posiible states as arry

    public State getTargetState() {
        return targetState;
    }

    public State getLastState() {
        return lastState;
    }

    public void setTargetState(State state) {
        if (systemStatesList.contains(state)) {
            lastState = targetState;
            targetState = state;
        } else {
            DriverStation.reportError("Cant set target state for " + getSubsystem(), true);
            targetState = StatesConstants.IDLE;
        }
    }

    public boolean canMove() {
        return systemCanMove;
    }

    public State getCurrenState() {
        return targetState;
    }

    @Override
    public void periodic() {
        currentStateLog.update(getCurrenState().getName());
        systemFunctionStateLog.update(getSystemFunctionState().getName());
        targetStateLog.update(getTargetState().getName());
        systemCanMoveLog.update(canMove());

        if (board.getBoolean(systemName + " Manuel")) {
            setSystemFunctionState(StatesConstants.MANUEL);
        } else {
            setSystemFunctionState(StatesConstants.AUTOMATIC);
        }
    }

}
