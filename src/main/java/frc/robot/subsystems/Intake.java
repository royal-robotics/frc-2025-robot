package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
    private final SparkMax pivot = new SparkMax(1, MotorType.kBrushless);
    private final SparkMax pivotFollow = new SparkMax(2, MotorType.kBrushless);
    private final TalonFX intake = new TalonFX(12);
    private final TalonFX scorer = new TalonFX(14);
    private final CANcoder pivotEncoder = new CANcoder(6);

    private final SparkMaxConfig pivotConfig = new SparkMaxConfig();
    private final SparkMaxConfig pivotConfigFollow = new SparkMaxConfig();
    private final MotorOutputConfigs outputConfigs = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake);
    private final CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(80))
        .withStatorCurrentLimitEnable(true);
    private final Slot0Configs IntakePIDConfigs = new Slot0Configs()
        .withKP(20)
        .withKI(0)
        .withKD(0);
    private final Slot0Configs scorerPIDConfigs = new Slot0Configs()
        .withKP(20)
        .withKI(0)
        .withKD(0);
    private final MagnetSensorConfigs encoderConfigs = new MagnetSensorConfigs()
        .withMagnetOffset(Degrees.of(77.1))
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive);

    private final double pivotGearRatio = 88.89;

    private final RelativeEncoder pivotPosition;
    private final RelativeEncoder pivotPositionFollow;
    private final SparkClosedLoopController pivotController;
    private final StatusSignal<Angle> intakePosition;
    private final StatusSignal<Voltage> intakeVoltage;
    private final StatusSignal<Angle> scorerPosition;
    private final StatusSignal<Voltage> scorerVoltage;
    private final StatusSignal<Angle> pivotEncoderPosition;

    private final VoltageOut voltageRequest = new VoltageOut(Volts.of(0));
    private final PositionVoltage positionRequest = new PositionVoltage(Rotations.of(0.0));

    private final AnalogInput coral = new AnalogInput(0);

    //private final Angle pivotMinPosition = Degrees.of(-33.0);
    //private final Angle pivotMaxPosition = Degrees.of(91.0);

    private boolean pivotOverride = false;
    private double pivotOverrideValue = 0.0;

    public Intake() {
        pivotConfig
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .smartCurrentLimit(80);
        pivotConfig.closedLoop
            .p(1.0)
            .i(0.0)
            .d(0.0)
            .outputRange(-0.5,0.5);
        pivot.configure(pivotConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        pivotConfigFollow
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(80)
            .follow(pivot, true);
        pivotConfigFollow.closedLoop
            .p(1.0)
            .i(0.0)
            .d(0.0)
            .outputRange(-0.5,0.5);
        pivotFollow.configure(pivotConfigFollow,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        intake.getConfigurator().apply(outputConfigs);
        intake.getConfigurator().apply(currentConfigs);
        intake.getConfigurator().apply(IntakePIDConfigs);

        scorer.getConfigurator().apply(outputConfigs);
        scorer.getConfigurator().apply(currentConfigs);
        scorer.getConfigurator().apply(scorerPIDConfigs);

        pivotEncoder.getConfigurator().apply(encoderConfigs);

        pivotPosition = pivot.getEncoder();
        pivotPositionFollow = pivotFollow.getEncoder();
        pivotController = pivot.getClosedLoopController();
        intakePosition = intake.getPosition();
        intakeVoltage = intake.getMotorVoltage();
        scorerPosition = scorer.getPosition();
        scorerVoltage = scorer.getMotorVoltage();
        pivotEncoderPosition = pivotEncoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(50,
            intakePosition, intakeVoltage, scorerPosition, scorerVoltage,
            pivotEncoderPosition);
        ParentDevice.optimizeBusUtilizationForAll(
            intake, scorer, pivotEncoder);

        intakePosition.waitForUpdate(0.02);
        intake.setControl(positionRequest.withPosition(intakePosition.getValue()));

        scorerPosition.waitForUpdate(0.02);
        scorer.setControl(positionRequest.withPosition(scorerPosition.getValue()));

        pivotEncoderPosition.waitForUpdate(0.02);
        pivotPosition.setPosition(pivotEncoderPosition.getValue().in(Rotations) * pivotGearRatio);
        pivotPositionFollow.setPosition(pivotEncoderPosition.getValue().in(Rotations) * pivotGearRatio);

        SmartDashboard.putData("IntakeCoast", setCoast());
        SmartDashboard.putData("IntakeBrake", setBrake());
        SmartDashboard.putBoolean("PivotOverride", pivotOverride);
        SmartDashboard.putNumber("PivotOverrideValue", pivotOverrideValue);
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

    public double intakePosition() {
        return intakePosition.getValue().in(Rotations);
    }

    public double intakeVoltage() {
        return intakeVoltage.getValue().in(Volts);
    }

    public double scorerPosition() {
        return scorerPosition.getValue().in(Rotations);
    }

    public double scorerVoltage() {
        return scorerVoltage.getValue().in(Volts);
    }

    public double pivotEncoderPosition() {
        return pivotEncoderPosition.getValue().in(Degrees);
    }

    public boolean hasCoral() {
        return coral.getAverageVoltage() < 2.5;
    }

    public Command setPivotForward() {
        return startEnd(
            () -> pivot.setVoltage(Volts.of(3)),
            () -> pivot.setVoltage(Volts.of(0))
        );
    }

    public Command setPivotBackward() {
        return startEnd(
            () -> pivot.setVoltage(Volts.of(-3)),
            () -> pivot.setVoltage(Volts.of(0))
        );
    }

    public Command setIntakeForward() {
        return startEnd(
            () -> intake.setControl(voltageRequest.withOutput(Volts.of(3))),
            () -> intake.setControl(voltageRequest.withOutput(Volts.of(0)))
        );
    }

    public Command setIntakeBackward() {
        return startEnd(
            () -> intake.setControl(voltageRequest.withOutput(Volts.of(-3))),
            () -> intake.setControl(voltageRequest.withOutput(Volts.of(0)))
        );
    }

    public Command setScorerPosition() {
        return runOnce(
            () -> scorer.setControl(positionRequest.withPosition(scorerPosition.getValue()))
        );
    }

    public Command setScorerForward() {
        return startEnd(
            () -> scorer.setControl(voltageRequest.withOutput(Volts.of(3.0))),
            () -> scorer.setControl(positionRequest.withPosition(scorerPosition.getValue()))
        );
    }

    public Command scorerForward() {
        return runOnce(
            () -> scorer.setControl(voltageRequest.withOutput(Volts.of(3.0)))
        );
    }

    public Command setScorerBackward() {
        return startEnd(
            () -> scorer.setControl(voltageRequest.withOutput(Volts.of(-3.0))),
            () -> scorer.setControl(positionRequest.withPosition(scorerPosition.getValue()))
        );
    }

    public Command setScorerBackwardSlow() {
        return startEnd(
            () -> scorer.setControl(voltageRequest.withOutput(Volts.of(-1.5))),
            () -> scorer.setControl(positionRequest.withPosition(scorerPosition.getValue()))
        );
    }

    public Command setScorerBackwardVoltage() {
        return startEnd(
            () -> scorer.setControl(voltageRequest.withOutput(Volts.of(-3.0))),
            () -> scorer.setControl(voltageRequest.withOutput(Volts.of(0.0)))
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
            scorer.setNeutralMode(NeutralModeValue.Brake);
        }).ignoringDisable(true);
    }

    public void periodic() {
        BaseStatusSignal.refreshAll(
            intakePosition, intakeVoltage, scorerPosition, scorerVoltage, pivotEncoderPosition);

        if (SmartDashboard.getBoolean("PivotOverride", pivotOverride)) {
            pivotController.setReference(
                SmartDashboard.getNumber("PivotOverrideValue", pivotOverrideValue),
                ControlType.kPosition);
        } else {
            pivotOverrideValue = pivotPosition();
            SmartDashboard.putNumber("PivotOverrideValue", pivotOverrideValue);
        }
    }

    public Command intakeAlgae() {
        return startEnd(
            () -> {
                pivotController.setReference(-1.8, ControlType.kPosition);
                intake.setControl(voltageRequest.withOutput(3));
            },
            () -> {
                intake.setControl(positionRequest.withPosition(intakePosition.getValue()));
                pivotController.setReference(-20.5, ControlType.kPosition);
            }
        );
    }

    public Command scoreAlgae() {
        return startEnd(
            () -> intake.setControl(voltageRequest.withOutput(-3.0)),
            () -> intake.setControl(positionRequest.withPosition(intakePosition.getValue()))
        );
    }
}
