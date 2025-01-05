// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.ma5951.utils.InDeveleop;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class PowerControlledTalonFX {
    private TalonFX motor;
    private TalonFXConfiguration configuration;

    public PowerControlledTalonFX(int ID ) {
        motor = new TalonFX(ID);
        configuration = new TalonFXConfiguration();
        configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motor.setSafetyEnabled(true);
        applyConfiguration();
    }

    public PowerControlledTalonFX(int ID , String canBus) {
        motor = new TalonFX(ID, canBus);
        configuration = new TalonFXConfiguration();
        configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motor.setSafetyEnabled(true);
        applyConfiguration();
    }

    public PowerControlledTalonFX(int ID , TalonFXConfiguration config) {
        motor = new TalonFX(ID);
        motor.getConfigurator().apply(config);
    }

    public PowerControlledTalonFX(int ID , String canBus , TalonFXConfiguration config) {
        motor = new TalonFX(ID , canBus);
        motor.getConfigurator().apply(config);
    }

    public void applyConfiguration() {
        motor.getConfigurator().apply(configuration);
    }

    public void setInverted(Boolean isInverted) {
        if (isInverted) {
            configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            applyConfiguration();
        } else {
            configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            applyConfiguration();
        }
    }

    public void setNutralMode(Boolean isBrake) {
        if (isBrake) {
            configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            applyConfiguration();
        } else {
            configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            applyConfiguration();
        }
    }

    public void setContinuosCurrentLimit(double continuosCurrent) {
        configuration.CurrentLimits.StatorCurrentLimit = continuosCurrent;
        configuration.CurrentLimits.StatorCurrentLimitEnable = true;

        applyConfiguration();
    }

    public void setPeakCurrentLimit(double peakCurrent , double peakTime) {
        // configuration.CurrentLimits.SupplyCurrentThreshold = peakCurrent;
        // configuration.CurrentLimits.SupplyTimeThreshold = peakTime;
        // configuration.CurrentLimits.SupplyCurrentLimitEnable = true; //TODO

        applyConfiguration();
    }

    public TalonFXConfiguration getConfig() {
        return configuration;
    }

    public TalonFX getMotor() {
        return motor;
    }

    public double getCurrent() {
        return motor.getStatorCurrent().refresh().getValueAsDouble();
    }

    public void disableSaftey() {
        motor.setSafetyEnabled(false);
    }

    public void setPower(double power) {
        motor.set(power);
    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);;
    }
    
}
