package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


@Logged
public class Intake extends SubsystemBase {
    private final SparkMax pivot1 = new SparkMax(1, MotorType.kBrushless);
    private final SparkMax pivot2 = new SparkMax(2, MotorType.kBrushless);

    private final RelativeEncoder pivotPosition1;
    private final RelativeEncoder pivotPosition2;

    public Intake() {
        pivotPosition1 = pivot1.getEncoder();
        pivotPosition2 = pivot2.getEncoder();
    }

    public double pivot1Position() {
        return pivotPosition1.getPosition();
    }
    public double pivot2Position() {
        return pivotPosition2.getPosition();
    }

    public Command setVoltage1() {
        return this.startEnd(
            () -> pivot1.setVoltage(Volts.of(3)),
            () -> pivot1.setVoltage(Volts.of(0))
        );
    }

    public Command setVoltage2() {
        return this.startEnd(
            () -> pivot2.setVoltage(Volts.of(3)),
            () -> pivot2.setVoltage(Volts.of(0))
        );
    }

    public void periodic() {
    }
}
