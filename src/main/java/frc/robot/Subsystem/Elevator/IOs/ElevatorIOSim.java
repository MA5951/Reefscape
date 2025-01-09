
package frc.robot.Subsystem.Elevator.IOs;

import com.ma5951.utils.ControlledMotors.Sim.TalonFXMotorSim;

import edu.wpi.first.math.system.plant.DCMotor;

public class ElevatorIOSim extends ElevatorIOReal{

    private TalonFXMotorSim elevatorMotorSim;

    public ElevatorIOSim() {
        super();
        elevatorMotorSim = new TalonFXMotorSim(masterMotor, masterConfig, DCMotor.getKrakenX60(2), 0.8, false);
    }

    @Override
    public void updatePeriodic() {
        super.updatePeriodic();
        elevatorMotorSim.updateSim();
    }
}
