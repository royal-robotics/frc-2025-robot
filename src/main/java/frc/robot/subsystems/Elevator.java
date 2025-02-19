package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Elevator extends SubsystemBase {
    private final TalonFX elevator1 = new TalonFX(9);
    private final TalonFX elevator2 = new TalonFX(10);

    private StatusSignal<Angle> elevatorPosition1;
    private StatusSignal<Voltage> elevatorVoltage1;
    private StatusSignal<Angle> elevatorPosition2;
    private StatusSignal<Voltage> elevatorVoltage2;

    private final VoltageOut  elevatorVoltageRequest = new VoltageOut(Volts.of(0));

    public Elevator() {
        elevator1.getConfigurator().apply(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake));
        elevator2.getConfigurator().apply(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake));

        elevatorPosition1 = elevator1.getPosition();
        elevatorPosition2 = elevator2.getPosition();
        elevatorVoltage1 = elevator1.getMotorVoltage();
        elevatorVoltage2 = elevator2.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(50,
            elevatorPosition1, elevatorPosition2, elevatorVoltage1, elevatorVoltage2);

        ParentDevice.optimizeBusUtilizationForAll(
            elevator1, elevator2);
    }

    public double elevatorPosition1() {
        return elevatorPosition1.getValue().in(Rotations);
    }

    public double elevatorVoltage1() {
        return elevatorVoltage1.getValue().in(Volts);
    }

    public double elevatorPosition2() {
        return elevatorPosition2.getValue().in(Rotations);
    }

    public double elevatorVoltage2() {
        return elevatorVoltage2.getValue().in(Volts);
    }

    public void periodic() {
        BaseStatusSignal.refreshAll(
            elevatorPosition1, elevatorPosition2, elevatorVoltage1, elevatorVoltage2);
    }

public Command setVoltageForward1() {
        return this.startEnd(
            () -> elevator1.setControl(elevatorVoltageRequest.withOutput(Volts.of(2))),
            () -> elevator1.setControl(elevatorVoltageRequest.withOutput(Volts.of(0)))
        );
    }

    public Command setVoltageBackward1() {
        return this.startEnd(
            () -> elevator1.setControl(elevatorVoltageRequest.withOutput(Volts.of(-2))),
            () -> elevator1.setControl(elevatorVoltageRequest.withOutput(Volts.of(0)))
        );
    }

    public Command setVoltageForward2() {
        return this.startEnd(
            () -> elevator2.setControl(elevatorVoltageRequest.withOutput(Volts.of(2))),
            () -> elevator2.setControl(elevatorVoltageRequest.withOutput(Volts.of(0)))
        );
    }

    public Command setVoltageBackward2() {
        return this.startEnd(
            () -> elevator2.setControl(elevatorVoltageRequest.withOutput(Volts.of(-2))),
            () -> elevator2.setControl(elevatorVoltageRequest.withOutput(Volts.of(0)))
        );
    }

}
