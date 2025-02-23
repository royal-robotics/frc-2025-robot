package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogInput;
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
    private final CANcoder armEncoder = new CANcoder(7);
    private final CANcoder wristEncoder = new CANcoder(8);

    private final MotorOutputConfigs outputConfigsInverted = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive);
    private final CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(80))
        .withStatorCurrentLimitEnable(true);
    private final Slot0Configs elevatorPIDConfigs = new Slot0Configs()
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withKG(0.2)
        .withKP(2.0)
        .withKI(0)
        .withKD(0);
    private final Slot0Configs armPIDConfigs = new Slot0Configs()
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withKG(0.2)
        .withKP(80.0)
        .withKI(0)
        .withKD(0);
    private final Slot0Configs wristPIDConfigs = new Slot0Configs()
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withKG(0.2)
        .withKP(80.0)
        .withKI(0)
        .withKD(0);
    private final FeedbackConfigs armFeedbackConfigs = new FeedbackConfigs()
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
        .withFeedbackRemoteSensorID(7);
    private final FeedbackConfigs wristFeedbackConfigs = new FeedbackConfigs()
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
        .withFeedbackRemoteSensorID(8);
    private final MagnetSensorConfigs armEncoderConfigs = new MagnetSensorConfigs()
        .withMagnetOffset(Degrees.of(-116.0));
    private final MagnetSensorConfigs wristEncoderConfigs = new MagnetSensorConfigs()
        .withMagnetOffset(Degrees.of(38.0));

    private final StatusSignal<Angle> elevatorPosition;
    private final StatusSignal<Voltage> elevatorVoltage;
    private final StatusSignal<Angle> armPosition;
    private final StatusSignal<Voltage> armVoltage;
    private final StatusSignal<Angle> wristPosition;
    private final StatusSignal<Voltage> wristVoltage;
    private final StatusSignal<Angle> armEncoderPosition;
    private final StatusSignal<Angle> wristEncoderPosition;

    private final VoltageOut voltageRequest = new VoltageOut(Volts.of(0));
    private final Follower followRequest = new Follower(9, false);
    private final PositionVoltage positionRequest = new PositionVoltage(Degrees.of(0.0));

    private final DigitalInput bottom = new DigitalInput(0);
    private final AnalogInput coral = new AnalogInput(0);

    private final double elevatorRotationsToInches = 1.0;

    private boolean elevatorOverride = false;
    private double elevatorOverrideValue = 0.0;
    private boolean armOverride = false;
    private double armOverrideValue = 0.0;
    private boolean wristOverride = false;
    private double wristOverrideValue = 0.0;

    public Elevator() {
        elevator.getConfigurator().apply(outputConfigsInverted);
        elevator.getConfigurator().apply(currentConfigs);
        elevator.getConfigurator().apply(elevatorPIDConfigs);
        elevatorFollow.getConfigurator().apply(outputConfigsInverted);
        elevatorFollow.getConfigurator().apply(currentConfigs);
        elevatorFollow.getConfigurator().apply(elevatorPIDConfigs);
        arm.getConfigurator().apply(outputConfigsInverted);
        arm.getConfigurator().apply(currentConfigs);
        arm.getConfigurator().apply(armFeedbackConfigs);
        arm.getConfigurator().apply(armPIDConfigs);
        wrist.getConfigurator().apply(outputConfigsInverted);
        wrist.getConfigurator().apply(currentConfigs);
        wrist.getConfigurator().apply(wristFeedbackConfigs);
        wrist.getConfigurator().apply(wristPIDConfigs);
        armEncoder.getConfigurator().apply(armEncoderConfigs);
        wristEncoder.getConfigurator().apply(wristEncoderConfigs);

        elevatorPosition = elevator.getPosition();
        elevatorVoltage = elevator.getMotorVoltage();
        armPosition = arm.getPosition();
        armVoltage = arm.getMotorVoltage();
        wristPosition = wrist.getPosition();
        wristVoltage = wrist.getMotorVoltage();
        armEncoderPosition = armEncoder.getAbsolutePosition();
        wristEncoderPosition = wristEncoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(50,
            elevatorPosition, elevatorVoltage, armPosition, armVoltage,
            wristPosition, wristVoltage, armEncoderPosition, wristEncoderPosition);
        //ParentDevice.optimizeBusUtilizationForAll(
        //    elevator, elevatorFollow, arm, wrist, armEncoder, wristEncoder);

        elevatorFollow.setControl(followRequest);

        SmartDashboard.putData("ElevatorCoast", setCoast());
        SmartDashboard.putData("ElevatorBrake", setBrake());
        SmartDashboard.putBoolean("ElevatorOverride", elevatorOverride);
        SmartDashboard.putNumber("ElevatorOverrideValue", elevatorOverrideValue);
        SmartDashboard.putBoolean("ArmOverride", armOverride);
        SmartDashboard.putNumber("ArmOverrideValue", armOverrideValue);
        SmartDashboard.putBoolean("WristOverride", wristOverride);
        SmartDashboard.putNumber("WristOverrideValue", wristOverrideValue);
    }

    public double elevatorPosition() {
        return elevatorPosition.getValue().in(Rotations);
    }

    public double elevatorHeight() {
        return elevatorPosition() * elevatorRotationsToInches;
    }

    public double elevatorVoltage() {
        return elevatorVoltage.getValue().in(Volts);
    }

    public double armPosition() {
        return armPosition.getValue().in(Degrees);
    }

    public double armVoltage() {
        return armVoltage.getValue().in(Volts);
    }

    public double wristPosition() {
        return wristPosition.getValue().in(Degrees);
    }

    public double wristVoltage() {
        return wristVoltage.getValue().in(Volts);
    }

    public boolean bottomLimit() {
        return bottom.get();
    }

    public double coralLimit() {
        return coral.getAverageVoltage();
    }

    public Command setElevatorForward() {
        return startEnd(
            () -> elevator.setControl(voltageRequest.withOutput(Volts.of(1))),
            () -> elevator.setControl(voltageRequest.withOutput(Volts.of(0)))
        );
    }

    public Command setElevatorBackward() {
        return startEnd(
            () -> elevator.setControl(voltageRequest.withOutput(Volts.of(-1))),
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
            wristPosition, wristVoltage, armEncoderPosition, wristEncoderPosition);

        if (SmartDashboard.getBoolean("ElevatorOverride", elevatorOverride)) {
            elevator.setControl(positionRequest.withPosition(Rotations.of(
                SmartDashboard.getNumber("ElevatorOverrideValue", elevatorOverrideValue)
                / elevatorRotationsToInches)));
        } else {
            elevatorOverrideValue = elevatorHeight();
            SmartDashboard.putNumber("ElevatorOverrideValue", elevatorOverrideValue);
        }

        if (SmartDashboard.getBoolean("ArmOverride", armOverride)) {
            arm.setControl(positionRequest.withPosition(Degrees.of(
                SmartDashboard.getNumber("ArmOverrideValue", armOverrideValue))));
        } else {
            armOverrideValue = armPosition();
            SmartDashboard.putNumber("ArmOverrideValue", armOverrideValue);
        }

        if (SmartDashboard.getBoolean("WristOverride", wristOverride)) {
            wrist.setControl(positionRequest.withPosition(Degrees.of(
                SmartDashboard.getNumber("WristOverrideValue", wristOverrideValue))));
        } else {
            wristOverrideValue = wristPosition();
            SmartDashboard.putNumber("WristOverrideValue", wristOverrideValue);
        }
    }
}
