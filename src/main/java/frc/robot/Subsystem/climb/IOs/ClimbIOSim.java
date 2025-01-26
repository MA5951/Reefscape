
package frc.robot.Subsystem.climb.IOs;

import com.ma5951.utils.ControlledMotors.Sim.TalonFXMotorSim;

import edu.wpi.first.math.system.plant.DCMotor;

public class ClimbIOSim extends ClimbIOReal{

    private TalonFXMotorSim masterMotorSim;
    private TalonFXMotorSim slaveMotorSim;

    public ClimbIOSim() {
        super();
        masterMotorSim = new TalonFXMotorSim(masterMotor, masterConfig, DCMotor.getKrakenX60(1), 0, false);//TODO:I dont know
        masterMotorSim = new TalonFXMotorSim(slaveMotor, masterConfig, DCMotor.getKrakenX60(1), 0, false);//TODO:I dont know
    }

    @Override
    public void updatePeriodic() {
        super.updatePeriodic();
        masterMotorSim.updateSim();
        slaveMotorSim.updateSim();
    }
}
