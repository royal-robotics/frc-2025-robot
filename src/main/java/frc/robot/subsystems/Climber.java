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
public class Climber extends SubsystemBase {
    private final TalonFX climbMotor = new TalonFX(11);
    
    private final StatusSignal<Angle> climbMotorPosition;
    private final StatusSignal<Voltage> climbMotorVoltage;

    private final VoltageOut climbVoltageRequest = new VoltageOut(Volts.of(0));

    public Climber() {
        climbMotor.getConfigurator().apply(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake));

        climbMotorPosition = climbMotor.getPosition();
        climbMotorVoltage = climbMotor.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(50,
            climbMotorPosition, climbMotorVoltage);

        ParentDevice.optimizeBusUtilizationForAll(
            climbMotor);
    }

    public double climberPosition() {
        return climbMotorPosition.getValue().in(Rotations);
    }

    public double climberVoltage() {
        return climbMotorVoltage.getValue().in(Volts);
    }

    public Command setVoltageForward() {
        return this.startEnd(
            () -> climbMotor.setControl(climbVoltageRequest.withOutput(Volts.of(1))),
            () -> climbMotor.setControl(climbVoltageRequest.withOutput(Volts.of(0)))
        );
    }

    public Command setVoltageBack() {
        return this.startEnd(
            () -> climbMotor.setControl(climbVoltageRequest.withOutput(Volts.of(-1))),
            () -> climbMotor.setControl(climbVoltageRequest.withOutput(Volts.of(0)))
        );
    }

    public void periodic() {
        BaseStatusSignal.refreshAll(
            climbMotorPosition, climbMotorVoltage);
    }
}
