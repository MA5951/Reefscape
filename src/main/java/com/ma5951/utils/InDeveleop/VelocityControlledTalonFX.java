// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.InDeveleop;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.DriverStation;


/** Add your docs here. */
public class VelocityControlledTalonFX extends PowerControlledTalonFX{
    
    private VelocityVoltage velocityVoltage;
    private Double convertionFactor = 1d ;
    private Double tolorance = 50d;
    private Double setPoint = 0d;
    //private boolean isFalcon = false;
    private int maxSpeed = 6000;

    public VelocityControlledTalonFX(int ID , boolean isFalcon) {
        super(ID);
        velocityVoltage = new VelocityVoltage(0).withSlot(0);
        if (isFalcon) {
            maxSpeed = 6200;
        }
    }

    public VelocityControlledTalonFX(int ID , String canBus , boolean isFalcon) {
       super(ID , canBus);
       velocityVoltage = new VelocityVoltage(0);
       if (isFalcon) {
            maxSpeed = 6200;
        }
    }

    public VelocityControlledTalonFX(int ID , TalonFXConfiguration config , boolean isFalcon) {
        super(ID, config);
        velocityVoltage = new VelocityVoltage(0);
        if (isFalcon) {
            maxSpeed = 6200;
        }
    }

    public VelocityControlledTalonFX(int ID , String canBus , TalonFXConfiguration config , boolean isFalcon) {
       super(ID, canBus, config);
       velocityVoltage = new VelocityVoltage(0);
       if (isFalcon) {
            maxSpeed = 6200;
        }
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

    public void calculate(double setpoint) {
        if (setpoint > maxSpeed ) {
            DriverStation.reportError("Requested speed too high, ID: "+super.getMotor().getDeviceID(), false);
        } else {
            super.getMotor().setControl(
            velocityVoltage.withVelocity((setPoint / convertionFactor) / 60).withSlot(0)
        );
        setPoint = setpoint;
        }
        
    }

    public double getVelocity() {
        return (super.getMotor().getVelocity().getValueAsDouble() * convertionFactor) * 60;
    }

    public boolean isInTolorance() {
        return Math.abs(setPoint - getVelocity()) <= tolorance;
    }

}
