package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Intake extends SubsystemBase {
    private final TalonFX scorer = new TalonFX(14);

    private final MotorOutputConfigs outputConfigs = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake);
    private final CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(100))
        .withStatorCurrentLimitEnable(true);
    private final Slot0Configs scorerPIDConfigs = new Slot0Configs()
        .withKP(30)
        .withKI(0)
        .withKD(0);

    private final StatusSignal<Angle> scorerPosition;
    private final StatusSignal<Voltage> scorerVoltage;

    private final VoltageOut voltageRequest = new VoltageOut(Volts.of(0));
    private final PositionVoltage positionRequest = new PositionVoltage(Rotations.of(0.0));

    private final AnalogInput coral = new AnalogInput(0);

    public Intake() {
        scorer.getConfigurator().apply(outputConfigs);
        scorer.getConfigurator().apply(currentConfigs);
        scorer.getConfigurator().apply(scorerPIDConfigs);

        scorerPosition = scorer.getPosition();
        scorerVoltage = scorer.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(50,
            scorerPosition, scorerVoltage);
        ParentDevice.optimizeBusUtilizationForAll(
            scorer);

        scorerPosition.waitForUpdate(0.02);
        scorer.setControl(positionRequest.withPosition(scorerPosition.getValue()));

        SmartDashboard.putData("IntakeCoast", setCoast());
        SmartDashboard.putData("IntakeBrake", setBrake());
    }

    public double scorerPosition() {
        return scorerPosition.getValue().in(Rotations);
    }

    public double scorerVoltage() {
        return scorerVoltage.getValue().in(Volts);
    }

    public boolean hasCoral() {
        return coral.getAverageVoltage() < 2.5;
    }

    public Command setScorerPosition() {
        return runOnce(
            () -> scorer.setControl(positionRequest.withPosition(scorerPosition.getValue()))
        );
    }

    public Command runScorer() {
        return run(
            () -> scorer.setControl(voltageRequest.withOutput(Volts.of(-3.0)))
        );
    }

    public Command scoreCoralL1() {
        return startEnd(
            () -> scorer.setControl(voltageRequest.withOutput(Volts.of(1.5))),
            () -> scorer.setControl(positionRequest.withPosition(scorerPosition.getValue()))
        );
    }

    public Command removeAlgae() {
        return runOnce(
            () -> scorer.setControl(voltageRequest.withOutput(Volts.of(3.0)))
        );
    }

    public Command handleCoral() {
        return startEnd(
            () -> scorer.setControl(voltageRequest.withOutput(Volts.of(-3.0))),
            () -> scorer.setControl(positionRequest.withPosition(scorerPosition.getValue()))
        );
    }

    public Command scoreCoralL3() {
        return startEnd(
            () -> scorer.setControl(voltageRequest.withOutput(Volts.of(-2.0))),
            () -> scorer.setControl(positionRequest.withPosition(scorerPosition.getValue()))
        );
    }

    public Command handleCoralWithSensor() {
        return startEnd(
            () -> scorer.setControl(voltageRequest.withOutput(Volts.of(-3.0))),
            () -> scorer.setControl(voltageRequest.withOutput(Volts.of(0.0)))
        ).until(() -> hasCoral());
    }

    public Command setCoast() {
        return runOnce(() -> {
            scorer.setNeutralMode(NeutralModeValue.Coast);
        }).ignoringDisable(true);
    }

    public Command setBrake() {
        return runOnce(() -> {
            scorer.setNeutralMode(NeutralModeValue.Brake);
        }).ignoringDisable(true);
    }

    public void periodic() {
        BaseStatusSignal.refreshAll(
            scorerPosition, scorerVoltage);
    }
}
