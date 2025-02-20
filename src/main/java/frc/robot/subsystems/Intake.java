package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Intake extends SubsystemBase {
    private final SparkMax pivot = new SparkMax(1, MotorType.kBrushless);
    private final SparkMax pivotFollow = new SparkMax(2, MotorType.kBrushless);

    private final SparkMaxConfig config = new SparkMaxConfig();
    private final SparkMaxConfig configFollow = new SparkMaxConfig();

    private final RelativeEncoder pivotPosition;
    private final RelativeEncoder pivotPositionFollow;

    public Intake() {
        pivot.configure(config
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .smartCurrentLimit(80),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        pivotFollow.configure(configFollow
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(80)
            .follow(pivot, true),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        pivotPosition = pivot.getEncoder();
        pivotPositionFollow = pivotFollow.getEncoder();

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

    public Command setVoltageForward() {
        return startEnd(
            () -> pivot.setVoltage(Volts.of(3)),
            () -> pivot.setVoltage(Volts.of(0))
        );
    }

    public Command setVoltageBackward() {
        return startEnd(
            () -> pivot.setVoltage(Volts.of(-3)),
            () -> pivot.setVoltage(Volts.of(0))
        );
    }

    public Command setCoast() {
        return runOnce(() -> {
            pivot.configure(config
                .idleMode(IdleMode.kCoast),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
            pivotFollow.configure(configFollow
                .idleMode(IdleMode.kCoast),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
        });
    }

    public Command setBrake() {
        return runOnce(() -> {
            pivot.configure(config
                .idleMode(IdleMode.kBrake),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
            pivotFollow.configure(configFollow
                .idleMode(IdleMode.kBrake),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
        });
    }

    public void periodic() {
    }
}
