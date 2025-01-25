package frc.robot.Subsystem.Arm.IOs;

import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ma5951.utils.ControlledMotors.Sim.TalonFXMotorSim;

import edu.wpi.first.math.system.plant.DCMotor;

public class ArmIOSim extends ArmIOReal {
    private TalonFXMotorSim armMotorSim;
    private CANcoderSimState canCoderSimstate;

    public  ArmIOSim() {
        super();
        canCoderSimstate = new CANcoderSimState(absEncoder);
        armMotorSim = new TalonFXMotorSim(armMotor, armConfig, DCMotor.getKrakenX60(1), 0.004, false);
    }

    @Override
    public void updatePeriodic() {
        canCoderSimstate.setSupplyVoltage(12);
        canCoderSimstate.setMagnetHealth(MagnetHealthValue.Magnet_Green);
        canCoderSimstate.addPosition(0);
        armMotorSim.updateSim();
        super.updatePeriodic();
        
    }
}
