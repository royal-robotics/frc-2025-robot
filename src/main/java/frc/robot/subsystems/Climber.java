package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

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

    private final MotorOutputConfigs outputConfigs = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake);
    private final CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(95))
        .withStatorCurrentLimitEnable(true);
    private final Slot0Configs pidConfigs = new Slot0Configs()
        .withKP(400.0)
        .withKI(0.0)
        .withKD(0.0);
    private final FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
        .withFeedbackRemoteSensorID(5);
    private final MagnetSensorConfigs encoderConfigs = new MagnetSensorConfigs()
        .withMagnetOffset(Degrees.of(-135))
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive);
    
    private final StatusSignal<Angle> climberPosition;
    private final StatusSignal<Voltage> climberVoltage;
    private final StatusSignal<Angle> climberEncoderPosition;

    private final VoltageOut voltageRequest = new VoltageOut(Volts.of(0));
    private final PositionVoltage positionRequest = new PositionVoltage(Degrees.of(0.0));

    private final double outAngle = 35.0;
    private final double inAngle = -105.0;
    private final double resetAngle = -42.0;

    public Climber() {
        climber.getConfigurator().apply(outputConfigs);
        climber.getConfigurator().apply(currentConfigs);
        climber.getConfigurator().apply(feedbackConfigs);
        climber.getConfigurator().apply(pidConfigs);
        climberEncoder.getConfigurator().apply(encoderConfigs);

        climberPosition = climber.getPosition();
        climberVoltage = climber.getMotorVoltage();
        climberEncoderPosition = climberEncoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(50,
            climberPosition, climberVoltage, climberEncoderPosition);
        ParentDevice.optimizeBusUtilizationForAll(
            climber, climberEncoder);


        SmartDashboard.putData("ClimberCoast", setCoast());
        SmartDashboard.putData("ClimberBrake", setBrake());
    }

    public double climberPosition() {
        return climberPosition.getValue().in(Degrees);
    }

    public double climberVoltage() {
        return climberVoltage.getValue().in(Volts);
    }

    public double climberEncoderPosition() {
        return climberEncoderPosition.getValue().in(Degrees);
    }

    public boolean climberAtSetpoint(double setpoint) {
        return Math.abs(climberPosition() - setpoint) < 5.0;
    }

    public Command moveClimberOut() {
        return run(() -> climber.setControl(
            positionRequest.withPosition(Degrees.of(outAngle))
        )).until(() -> climberAtSetpoint(outAngle));
    }

    public Command moveClimberIn() {
        return run(() -> climber.setControl(
            positionRequest.withPosition(Degrees.of(inAngle))
        )).until(() -> climberAtSetpoint(inAngle));
    }

    public Command moveClimberReset() {
        return run(() -> climber.setControl(
            positionRequest.withPosition(Degrees.of(resetAngle))
        )).until(() -> climberAtSetpoint(resetAngle));
    }

    public Command setClimberForward() {
        return startEnd(
            () -> climber.setControl(voltageRequest.withOutput(Volts.of(12))),
            () -> climber.setControl(voltageRequest.withOutput(Volts.of(0)))
        ).onlyWhile(() -> climberPosition() < 35.0);
    }

    public Command setClimberBackward() {
        return startEnd(
            () -> climber.setControl(voltageRequest.withOutput(Volts.of(-12))),
            () -> climber.setControl(voltageRequest.withOutput(Volts.of(0)))
        ).onlyWhile(() -> climberPosition() > -110);
    }

    public Command setCoast() {
        return runOnce(() -> {
            climber.setNeutralMode(NeutralModeValue.Coast);
        }).ignoringDisable(true);
    }

    public Command setBrake() {
        return runOnce(() -> {
            climber.setNeutralMode(NeutralModeValue.Brake);
        }).ignoringDisable(true);
    }

    public void periodic() {
        BaseStatusSignal.refreshAll(
            climberPosition, climberVoltage, climberEncoderPosition);
    }
}
