
package com.ma5951.utils.ControlledMotors.Sim;

import org.ironmaple.simulation.motorsims.SimulatedBattery;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TalonFXMotorSim {

    private TalonFXSimState motorSimState;
    private DCMotorSim physicshSim;

    public TalonFXMotorSim(TalonFX motor , TalonFXConfiguration motorConfig ,DCMotor motorType , double Inertia , boolean isRevers) {
        motorSimState = motor.getSimState();
        physicshSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motorType, Inertia, motorConfig.Feedback.SensorToMechanismRatio), motorType ,  0.5 , 0.5);
        SimulatedBattery.addElectricalAppliances(() -> motor.getSupplyCurrent().refresh().getValue());
        if (isRevers) {
            motorSimState.Orientation = ChassisReference.Clockwise_Positive;
        }

    }

    public void updateSim() {
        motorSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage().baseUnitMagnitude());
        physicshSim.setInputVoltage(motorSimState.getMotorVoltage());
        physicshSim.update(0.02);

        motorSimState.setRawRotorPosition(physicshSim.getAngularPositionRotations());
        motorSimState.setRotorVelocity(ConvUtil.RPMtoRPS(physicshSim.getAngularVelocityRPM()));
    }



}
