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
    private final TalonFX arm = new TalonFX(15);
    private final TalonFX wrist = new TalonFX(16);

    private final MotorOutputConfigs outputConfigs = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake);
    private final MotorOutputConfigs outputConfigsInverted = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive);
    private final CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(80))
        .withStatorCurrentLimitEnable(true);

    private final StatusSignal<Angle> elevatorPosition;
    private final StatusSignal<Voltage> elevatorVoltage;
    private final StatusSignal<Angle> armPosition;
    private final StatusSignal<Voltage> armVoltage;
    private final StatusSignal<Angle> wristPosition;
    private final StatusSignal<Voltage> wristVoltage;

    private final VoltageOut voltageRequest = new VoltageOut(Volts.of(0));
    private final Follower followRequest = new Follower(9, false);

    private final DigitalInput bottom = new DigitalInput(0);

    public Elevator() {
        elevator.getConfigurator().apply(outputConfigsInverted);
        elevator.getConfigurator().apply(currentConfigs);
        elevatorFollow.getConfigurator().apply(outputConfigsInverted);
        elevatorFollow.getConfigurator().apply(currentConfigs);
        arm.getConfigurator().apply(outputConfigs);
        arm.getConfigurator().apply(currentConfigs);
        wrist.getConfigurator().apply(outputConfigs);
        wrist.getConfigurator().apply(currentConfigs);

        elevatorPosition = elevator.getPosition();
        elevatorVoltage = elevator.getMotorVoltage();
        armPosition = arm.getPosition();
        armVoltage = arm.getMotorVoltage();
        wristPosition = wrist.getPosition();
        wristVoltage = wrist.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(50,
            elevatorPosition, elevatorVoltage, armPosition, armVoltage,
            wristPosition, wristVoltage);
        ParentDevice.optimizeBusUtilizationForAll(
            elevator, elevatorFollow, arm, wrist);

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

    public double armPosition() {
        return armPosition.getValue().in(Rotations);
    }

    public double armVoltage() {
        return armVoltage.getValue().in(Volts);
    }

    public double wristPosition() {
        return wristPosition.getValue().in(Rotations);
    }

    public double wristVoltage() {
        return wristVoltage.getValue().in(Volts);
    }

    public boolean bottomLimit() {
        return bottom.get();
    }

    public Command setElevatorForward() {
        return startEnd(
            () -> elevator.setControl(voltageRequest.withOutput(Volts.of(2))),
            () -> elevator.setControl(voltageRequest.withOutput(Volts.of(0)))
        );
    }

    public Command setElevatorBackward() {
        return startEnd(
            () -> elevator.setControl(voltageRequest.withOutput(Volts.of(-2))),
            () -> elevator.setControl(voltageRequest.withOutput(Volts.of(0)))
        );
    }

    public Command setArmForward() {
        return startEnd(
            () -> arm.setControl(voltageRequest.withOutput(Volts.of(1))),
            () -> arm.setControl(voltageRequest.withOutput(Volts.of(0)))
        );
    }

    public Command setArmBackward() {
        return startEnd(
            () -> arm.setControl(voltageRequest.withOutput(Volts.of(-1))),
            () -> arm.setControl(voltageRequest.withOutput(Volts.of(0)))
        );
    }

    public Command setWristForward() {
        return startEnd(
            () -> wrist.setControl(voltageRequest.withOutput(Volts.of(1))),
            () -> wrist.setControl(voltageRequest.withOutput(Volts.of(0)))
        );
    }

    public Command setWristBackward() {
        return startEnd(
            () -> wrist.setControl(voltageRequest.withOutput(Volts.of(-1))),
            () -> wrist.setControl(voltageRequest.withOutput(Volts.of(0)))
        );
    }

    public Command setCoast() {
        return runOnce(() -> {
            elevator.setNeutralMode(NeutralModeValue.Coast);
            elevatorFollow.setNeutralMode(NeutralModeValue.Coast);
            arm.setNeutralMode(NeutralModeValue.Coast);
            wrist.setNeutralMode(NeutralModeValue.Coast);
        }).ignoringDisable(true);
    }

    public Command setBrake() {
        return runOnce(() -> {
            elevator.setNeutralMode(NeutralModeValue.Brake);
            elevatorFollow.setNeutralMode(NeutralModeValue.Brake);
            arm.setNeutralMode(NeutralModeValue.Brake);
            wrist.setNeutralMode(NeutralModeValue.Brake);
        }).ignoringDisable(true);
    }

    public void periodic() {
        BaseStatusSignal.refreshAll(
            elevatorPosition, elevatorVoltage, armPosition, armVoltage,
            wristPosition, wristVoltage);
    }
}
