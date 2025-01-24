
package frc.robot.Subsystem.Elevator.IOs;


import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.RobotConstants;
import frc.robot.Subsystem.Elevator.ElevatorConstants;

public class ElevatorIOSim extends ElevatorIOReal{

    private TalonFXSimState simState;
    private ElevatorSim elevatorSim;

    public ElevatorIOSim() {
        super();
        simState = new TalonFXSimState(masterMotor);
        elevatorSim = new ElevatorSim(DCMotor.getKrakenX60(2),   ElevatorConstants.GEAR, 
        ElevatorConstants.WEIGHT_OF_MOVING_PARTS, ElevatorConstants.SPROKET_PITCH_DIAMETER / 2, ElevatorConstants.MIN_HIGHT, 
        ElevatorConstants.MAX_HIGHT, false, ElevatorConstants.MIN_HIGHT, 0, 0);
        
    }

    @Override
    public void updatePeriodic() {
        

        simState.setSupplyVoltage(12);


        elevatorSim.setInputVoltage(simState.getMotorVoltage());
        elevatorSim.update(RobotConstants.kDELTA_TIME);
        simState.setRawRotorPosition(elevatorSim.getPositionMeters() / ElevatorConstants.SPROKET_CIRCUMFERENCE * ElevatorConstants.GEAR);
        simState.setRotorVelocity(ConvUtil.RPMtoRPS((elevatorSim.getVelocityMetersPerSecond() / (Math.PI * ElevatorConstants.SPROKET_PITCH_DIAMETER)) * 60));

        super.updatePeriodic();
    }
}
