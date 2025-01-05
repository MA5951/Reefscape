// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.InDeveleop;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;

/** Add your docs here. */
public class PositionControlledTalonFX  extends PowerControlledTalonFX{

    private PositionVoltage positionVoltage;
    private StatusSignal<Double> pose;
    private Double convertionFactor = 0d ;
    private Double tolorance = 0d ;
    private Double setPoint = 0d;

    public PositionControlledTalonFX(int ID) {
        super(ID);
        positionVoltage = new PositionVoltage(0);
    }

    public PositionControlledTalonFX(int ID , String canBus) {
        super(ID, canBus);
        positionVoltage = new PositionVoltage(0);
    }

    public PositionControlledTalonFX(int ID , TalonFXConfiguration config) {
        super(ID, config);
        positionVoltage = new PositionVoltage(0);
    }

    public PositionControlledTalonFX(int ID , String canBus, TalonFXConfiguration config ) {
        super(ID , canBus , config);
        positionVoltage = new PositionVoltage(0);
    }
   
    public void setTolorance(double t) {
        tolorance = t;
    }

    public double getTolorance() {
        return tolorance;
    }

    public void setConvertionFactor(double factor) {
        convertionFactor = factor;
    }

    public double getConvertionFactor() {
        return convertionFactor;
    }

    public void setPID(double p , double i , double d) {
        super.getConfig().Slot0.kP = 0;
        super.getConfig().Slot0.kI = 0;
        super.getConfig().Slot0.kD = 0;

        applyConfiguration();
    }

    public void setFeedForward(double ks , double kv) {
        super.getConfig().Slot0.kS = ks;
        super.getConfig().Slot0.kV = kv;

        applyConfiguration();
    }

    public void calculate(double setPoint) {
        super.getMotor().setControl(
            positionVoltage.withPosition(setPoint * convertionFactor).withSlot(0)
        );
        this.setPoint = setPoint;
    }

    public double getPosition() {
        pose.refresh();
        return pose.getValueAsDouble() / convertionFactor;
    }

    public boolean isInTolorance() {
        return Math.abs(setPoint - getPosition()) <= tolorance;
    }

    public void setEncoder(double newPose) {
        super.getMotor().setPosition(newPose);
    }

}
