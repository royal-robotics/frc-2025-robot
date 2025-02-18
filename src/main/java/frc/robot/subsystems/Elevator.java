package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Elevator extends SubsystemBase {
    private final TalonFX mainElevator1 = new TalonFX(9);
    private final TalonFX mainElevator2 = new TalonFX(10);

    private StatusSignal<Angle> elevatorPosition1;
    private StatusSignal<Voltage> elevatorVoltage1;
    private StatusSignal<Angle> elevatorPosition2;
    private StatusSignal<Voltage> elevatorVoltage2;

    public Elevator () {
        elevatorPosition1 = mainElevator1.getPosition();
        elevatorPosition2 = mainElevator2.getPosition();
        elevatorVoltage1 = mainElevator1.getMotorVoltage();
        elevatorVoltage2 = mainElevator2.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(50,
            elevatorPosition1, elevatorPosition2, elevatorVoltage1, elevatorVoltage2);

        ParentDevice.optimizeBusUtilizationForAll(
            mainElevator1, mainElevator2);
    }

    public void periodic() {
        BaseStatusSignal.refreshAll(
            elevatorPosition1, elevatorPosition2, elevatorVoltage1, elevatorVoltage2);
    }
}
