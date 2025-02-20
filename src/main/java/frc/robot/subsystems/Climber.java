package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Climber extends SubsystemBase {
    private final TalonFX climber = new TalonFX(11);
    private final CANcoder climberEncoder = new CANcoder(5);
    private final Angle climberEncoderOffset = Degrees.of(0.0);

    private final MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    private final CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
    
    private final StatusSignal<Angle> climberPosition;
    private final StatusSignal<Voltage> climberVoltage;
    private final StatusSignal<Angle> climberEncoderPosition;

    private final VoltageOut climberVoltageRequest = new VoltageOut(Volts.of(0));

    public Climber() {
        climber.getConfigurator().apply(outputConfigs
            .withNeutralMode(NeutralModeValue.Coast));
        climber.getConfigurator().apply(currentConfigs
            .withStatorCurrentLimit(Amps.of(80))
            .withStatorCurrentLimitEnable(true));

        climberPosition = climber.getPosition();
        climberVoltage = climber.getMotorVoltage();
        climberEncoderPosition = climberEncoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(50,
            climberPosition, climberVoltage, climberEncoderPosition);
        ParentDevice.optimizeBusUtilizationForAll(
            climber);

        SmartDashboard.putData("ClimberCoast", setCoast());
        SmartDashboard.putData("ClimberBrake", setBrake());
    }

    public double climberPosition() {
        return climberPosition.getValue().in(Rotations);
    }

    public double climberVoltage() {
        return climberVoltage.getValue().in(Volts);
    }

    public double climberEncoderPosition() {
        return climberEncoderPosition.getValue().plus(climberEncoderOffset).in(Degrees);
    }

    public Command setVoltageForward() {
        return startEnd(
            () -> climber.setControl(climberVoltageRequest.withOutput(Volts.of(1))),
            () -> climber.setControl(climberVoltageRequest.withOutput(Volts.of(0)))
        );
    }

    public Command setVoltageBack() {
        return startEnd(
            () -> climber.setControl(climberVoltageRequest.withOutput(Volts.of(-1))),
            () -> climber.setControl(climberVoltageRequest.withOutput(Volts.of(0)))
        );
    }

    public Command setCoast() {
        return runOnce(() -> {
            climber.getConfigurator().apply(outputConfigs
                .withNeutralMode(NeutralModeValue.Coast));
        });
    }

    public Command setBrake() {
        return runOnce(() -> {
            climber.getConfigurator().apply(outputConfigs
                .withNeutralMode(NeutralModeValue.Brake));
        });
    }

    public void periodic() {
        BaseStatusSignal.refreshAll(
            climberPosition, climberVoltage, climberEncoderPosition);
    }
}
