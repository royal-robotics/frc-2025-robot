package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Elevator extends SubsystemBase {
    private final TalonFX elevator = new TalonFX(9);
    private final TalonFX elevatorFollow = new TalonFX(10);

    private final MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    private final CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();

    private final StatusSignal<Angle> elevatorPosition;
    private final StatusSignal<Voltage> elevatorVoltage;
    private final StatusSignal<Angle> elevatorPositionFollow;
    private final StatusSignal<Voltage> elevatorVoltageFollow;

    private final VoltageOut elevatorVoltageRequest = new VoltageOut(Volts.of(0));
    private final Follower followRequest = new Follower(9, false);

    private final DigitalInput bottom = new DigitalInput(0);

    public Elevator() {
        elevator.getConfigurator().apply(outputConfigs
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive));
        elevator.getConfigurator().apply(currentConfigs
            .withStatorCurrentLimit(Amps.of(80))
            .withStatorCurrentLimitEnable(true));
        elevatorFollow.getConfigurator().apply(outputConfigs
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive));
        elevatorFollow.getConfigurator().apply(currentConfigs
            .withStatorCurrentLimit(Amps.of(80))
            .withStatorCurrentLimitEnable(true));

        elevatorPosition = elevator.getPosition();
        elevatorPositionFollow = elevatorFollow.getPosition();
        elevatorVoltage = elevator.getMotorVoltage();
        elevatorVoltageFollow = elevatorFollow.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(50,
            elevatorPosition, elevatorPositionFollow, elevatorVoltage, elevatorVoltageFollow);
        ParentDevice.optimizeBusUtilizationForAll(
            elevator, elevatorFollow);

        elevatorFollow.setControl(followRequest);

        SmartDashboard.putData("ElevatorCoast", setCoast());
        SmartDashboard.putData("ElevatorBrake", setBrake());
    }

    public double elevatorPosition() {
        return elevatorPosition.getValue().in(Rotations);
    }

    public double elevatorVoltage() {
        return elevatorVoltage.getValue().in(Volts);
    }

    public double elevatorPositionFollow() {
        return elevatorPositionFollow.getValue().in(Rotations);
    }

    public double elevatorVoltageFollow() {
        return elevatorVoltageFollow.getValue().in(Volts);
    }

    public boolean bottomLimit() {
        return bottom.get();
    }

    public Command setVoltageForward() {
        return startEnd(
            () -> elevator.setControl(elevatorVoltageRequest.withOutput(Volts.of(2))),
            () -> elevator.setControl(elevatorVoltageRequest.withOutput(Volts.of(0)))
        );
    }

    public Command setVoltageBackward() {
        return startEnd(
            () -> elevator.setControl(elevatorVoltageRequest.withOutput(Volts.of(-2))),
            () -> elevator.setControl(elevatorVoltageRequest.withOutput(Volts.of(0)))
        );
    }

    public Command setCoast() {
        return runOnce(() -> {
            elevator.getConfigurator().apply(outputConfigs
                .withNeutralMode(NeutralModeValue.Coast));
            elevatorFollow.getConfigurator().apply(outputConfigs
                .withNeutralMode(NeutralModeValue.Coast));
        });
    }

    public Command setBrake() {
        return runOnce(() -> {
            elevator.getConfigurator().apply(outputConfigs
                .withNeutralMode(NeutralModeValue.Brake));
            elevatorFollow.getConfigurator().apply(outputConfigs
                .withNeutralMode(NeutralModeValue.Brake));
        });
    }

    public void periodic() {
        BaseStatusSignal.refreshAll(
            elevatorPosition, elevatorPositionFollow, elevatorVoltage, elevatorVoltageFollow);
    }
}
