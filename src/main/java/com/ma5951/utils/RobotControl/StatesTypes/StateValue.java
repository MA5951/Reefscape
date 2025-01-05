// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.RobotControl.StatesTypes;


public class StateValue extends State{

    private double value;

    public StateValue(String name , double variable) {
        super(name);
        value = variable;
    }

    public double getValue() {
        return value;
    }
}

