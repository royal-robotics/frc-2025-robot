package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
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

    private final MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    private final CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
    private final FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
    private final MagnetSensorConfigs encoderConfigs = new MagnetSensorConfigs();
    private final Angle climberEncoderOffset = Degrees.of(0.0);
    
    private final StatusSignal<Angle> climberPosition;
    private final StatusSignal<Voltage> climberVoltage;
    private final StatusSignal<Angle> climberEncoderPosition;

    private final VoltageOut voltageRequest = new VoltageOut(Volts.of(0));

    public Climber() {
        climber.getConfigurator().apply(outputConfigs
            .withNeutralMode(NeutralModeValue.Brake));
        climber.getConfigurator().apply(currentConfigs
            .withStatorCurrentLimit(Amps.of(80))
            .withStatorCurrentLimitEnable(true));
        climber.getConfigurator().apply(feedbackConfigs
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
            .withFeedbackRemoteSensorID(5));
        climberEncoder.getConfigurator().apply(encoderConfigs
            .withMagnetOffset(climberEncoderOffset)
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive));

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
        return climberPosition.getValue().in(Degrees);
    }

    public double climberVoltage() {
        return climberVoltage.getValue().in(Volts);
    }

    public Command setClimberForward() {
        return startEnd(
            () -> climber.setControl(voltageRequest.withOutput(Volts.of(1))),
            () -> climber.setControl(voltageRequest.withOutput(Volts.of(0)))
        );
    }

    public Command setClimberBackward() {
        return startEnd(
            () -> climber.setControl(voltageRequest.withOutput(Volts.of(-1))),
            () -> climber.setControl(voltageRequest.withOutput(Volts.of(0)))
        );
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
