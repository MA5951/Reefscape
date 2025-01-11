package frc.robot.Subsystem.Arm.IOs;

import com.ma5951.utils.ControlledMotors.Sim.TalonFXMotorSim;

import edu.wpi.first.math.system.plant.DCMotor;

public class ArmIOSim extends ArmIOReal {
    private TalonFXMotorSim armMotorSim;

    public  ArmIOSim() {
        super();
        armMotorSim = new TalonFXMotorSim(armMotor, armConfig, DCMotor.getFalcon500(1), 0.05, false);
    }

    @Override
    public void updatePeriodic() {
        super.updatePeriodic();
        armMotorSim.updateSim();
    }
}
