package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Intake extends SubsystemBase {
    private final SparkMax pivot = new SparkMax(1, MotorType.kBrushless);
    private final SparkMax pivotFollow = new SparkMax(2, MotorType.kBrushless);
    private final TalonFX intake = new TalonFX(12);
    private final TalonFX holder = new TalonFX(13);
    private final TalonFX scorer = new TalonFX(14);

    private final SparkBaseConfig pivotConfig = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .inverted(true)
        .smartCurrentLimit(80);
    private final SparkBaseConfig pivotConfigFollow = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(80)
        .follow(pivot);
    private final MotorOutputConfigs outputConfigs = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake);
    private final CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(80))
        .withStatorCurrentLimitEnable(true);

    private final RelativeEncoder pivotPosition;
    private final RelativeEncoder pivotPositionFollow;
    private final StatusSignal<Voltage> intakeVoltage;
    private final StatusSignal<Voltage> holderVoltage;
    private final StatusSignal<Voltage> scorerVoltage;

    private final VoltageOut voltageRequest = new VoltageOut(Volts.of(0));

    public Intake() {
        pivot.configure(pivotConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        pivotFollow.configure(pivotConfigFollow,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        intake.getConfigurator().apply(outputConfigs);
        intake.getConfigurator().apply(currentConfigs);
        holder.getConfigurator().apply(outputConfigs);
        holder.getConfigurator().apply(currentConfigs);
        scorer.getConfigurator().apply(outputConfigs);
        scorer.getConfigurator().apply(currentConfigs);

        pivotPosition = pivot.getEncoder();
        pivotPositionFollow = pivotFollow.getEncoder();
        intakeVoltage = intake.getMotorVoltage();
        holderVoltage = holder.getMotorVoltage();
        scorerVoltage = scorer.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(50,
            intakeVoltage, holderVoltage, scorerVoltage);
        ParentDevice.optimizeBusUtilizationForAll(
            intake, holder, scorer);

        SmartDashboard.putData("IntakeCoast", setCoast());
        SmartDashboard.putData("IntakeBrake", setBrake());
    }

    public double pivotPosition() {
        return pivotPosition.getPosition();
    }

    public double pivotOutput() {
        return pivot.getAppliedOutput();
    }

    public double pivotPositionFollow() {
        return pivotPositionFollow.getPosition();
    }

    public double pivotOutputFollow() {
        return pivotFollow.getAppliedOutput();
    }

    public double intakeVoltage() {
        return intakeVoltage.getValue().in(Volts);
    }

    public double holderVoltage() {
        return holderVoltage.getValue().in(Volts);
    }

    public double scorerVoltage() {
        return scorerVoltage.getValue().in(Volts);
    }

    public Command setPivotForward() {
        return startEnd(
            () -> pivot.setVoltage(Volts.of(1)),
            () -> pivot.setVoltage(Volts.of(0))
        );
    }

    public Command setPivotBackward() {
        return startEnd(
            () -> pivot.setVoltage(Volts.of(-1)),
            () -> pivot.setVoltage(Volts.of(0))
        );
    }

    public Command setIntakeForward() {
        return startEnd(
            () -> intake.setControl(voltageRequest.withOutput(Volts.of(6))),
            () -> intake.setControl(voltageRequest.withOutput(Volts.of(0)))
        );
    }

    public Command setIntakeBackward() {
        return startEnd(
            () -> intake.setControl(voltageRequest.withOutput(Volts.of(-6))),
            () -> intake.setControl(voltageRequest.withOutput(Volts.of(0)))
        );
    }

    public Command setHolderForward() {
        return startEnd(
            () -> holder.setControl(voltageRequest.withOutput(Volts.of(6))),
            () -> holder.setControl(voltageRequest.withOutput(Volts.of(0)))
        );
    }

    public Command setHolderBackward() {
        return startEnd(
            () -> holder.setControl(voltageRequest.withOutput(Volts.of(-6))),
            () -> holder.setControl(voltageRequest.withOutput(Volts.of(0)))
        );
    }

    public Command setScorerForward() {
        return startEnd(
            () -> scorer.setControl(voltageRequest.withOutput(Volts.of(6))),
            () -> scorer.setControl(voltageRequest.withOutput(Volts.of(0)))
        );
    }

    public Command setScorerBackward() {
        return startEnd(
            () -> scorer.setControl(voltageRequest.withOutput(Volts.of(-6))),
            () -> scorer.setControl(voltageRequest.withOutput(Volts.of(0)))
        );
    }

    public Command setCoast() {
        return runOnce(() -> {
            pivot.configure(pivotConfig
                .idleMode(IdleMode.kCoast),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
            pivotFollow.configure(pivotConfigFollow
                .idleMode(IdleMode.kCoast),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
            intake.setNeutralMode(NeutralModeValue.Coast);
            holder.setNeutralMode(NeutralModeValue.Coast);
            scorer.setNeutralMode(NeutralModeValue.Coast);
        }).ignoringDisable(true);
    }

    public Command setBrake() {
        return runOnce(() -> {
            pivot.configure(pivotConfig
                .idleMode(IdleMode.kBrake),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
            pivotFollow.configure(pivotConfigFollow
                .idleMode(IdleMode.kBrake),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
            intake.setNeutralMode(NeutralModeValue.Brake);
            holder.setNeutralMode(NeutralModeValue.Brake);
            scorer.setNeutralMode(NeutralModeValue.Brake);
        }).ignoringDisable(true);
    }

    public void periodic() {
        BaseStatusSignal.refreshAll(
            intakeVoltage, holderVoltage, scorerVoltage
        );
    }
}
