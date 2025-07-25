package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.VoltageConfigs;
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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

@Logged
public class Elevator extends SubsystemBase {
    private final TalonFX elevator = new TalonFX(9);
    private final TalonFX elevatorFollow = new TalonFX(10);
    private final TalonFX arm = new TalonFX(15);
    private final TalonFX wrist = new TalonFX(16);
    private final CANcoder armEncoder = new CANcoder(7);
    private final CANcoder wristEncoder = new CANcoder(8);

    private final MotorOutputConfigs outputConfigs = new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive);
    private final VoltageConfigs voltageConfigs = new VoltageConfigs()
        .withPeakForwardVoltage(Volts.of(12.0))
        .withPeakReverseVoltage(Volts.of(-6.0));
    private final VoltageConfigs armVoltageConfigs = new VoltageConfigs()
        .withPeakForwardVoltage(Volts.of(3.0))
        .withPeakReverseVoltage(Volts.of(-3.0));
    private final VoltageConfigs wristVoltageConfigs = new VoltageConfigs()
        .withPeakForwardVoltage(Volts.of(3.0))
        .withPeakReverseVoltage(Volts.of(-3.0));
    private final CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(40))
        .withStatorCurrentLimitEnable(true);
    private final CurrentLimitsConfigs elevatorCurrentConfigs = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(80))
        .withStatorCurrentLimitEnable(true);
    private final Slot0Configs elevatorPIDConfigs = new Slot0Configs()
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withKG(0.6)
        .withKP(5.0)
        .withKI(0)
        .withKD(0.1);
    private final Slot0Configs armPIDConfigs = new Slot0Configs()
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withKG(0.4)
        .withKP(80.0)
        .withKI(0)
        .withKD(2.0);
    private final Slot0Configs wristPIDConfigs = new Slot0Configs()
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withKG(0.3)
        .withKP(90.0)
        .withKI(0)
        .withKD(0.0);
    private final FeedbackConfigs armFeedbackConfigs = new FeedbackConfigs()
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
        .withFeedbackRemoteSensorID(7);
    private final FeedbackConfigs wristFeedbackConfigs = new FeedbackConfigs()
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
        .withFeedbackRemoteSensorID(8);
    private final MagnetSensorConfigs armEncoderConfigs = new MagnetSensorConfigs()
        .withMagnetOffset(Degrees.of(241.96))
        .withAbsoluteSensorDiscontinuityPoint(Degrees.of(200.0));
    private final MagnetSensorConfigs wristEncoderConfigs = new MagnetSensorConfigs()
        .withMagnetOffset(Degrees.of(38.0));

    private final StatusSignal<Angle> elevatorPosition;
    private final StatusSignal<AngularVelocity> elevatorVelocity;
    private final StatusSignal<Voltage> elevatorVoltage;
    private final StatusSignal<Temperature> elevatorTemp;
    private final StatusSignal<Angle> elevatorPositionFollow;
    private final StatusSignal<AngularVelocity> elevatorVelocityFollow;
    private final StatusSignal<Voltage> elevatorVoltageFollow;
    private final StatusSignal<Angle> armPosition;
    private final StatusSignal<AngularVelocity> armVelocity;
    private final StatusSignal<Voltage> armVoltage;
    private final StatusSignal<Angle> wristPosition;
    private final StatusSignal<AngularVelocity> wristVelocity;
    private final StatusSignal<Voltage> wristVoltage;
    private final StatusSignal<Angle> armEncoderPosition;
    private final StatusSignal<AngularVelocity> armEncoderVelocity;
    private final StatusSignal<Angle> wristEncoderPosition;
    private final StatusSignal<AngularVelocity> wristEncoderVelocity;

    private final VoltageOut voltageRequest = new VoltageOut(Volts.of(0));
    private final Follower followRequest = new Follower(9, false);
    private final PositionVoltage positionRequest = new PositionVoltage(Degrees.of(0.0));

    private final SysIdRoutine sysIdElevator = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            null, // Use default step voltage output (7 V)
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdElevator_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> elevator.setControl(voltageRequest.withOutput(output)),
            null,
            this
        )
    );

    private final SysIdRoutine sysIdArm = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            null, // Use default step voltage output (7 V)
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdArm_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> arm.setControl(voltageRequest.withOutput(output)),
            null,
            this
        )
    );

    private final SysIdRoutine sysIdWrist = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            null, // Use default step voltage output (7 V)
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdWrist_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> wrist.setControl(voltageRequest.withOutput(output)),
            null,
            this
        )
    );

    private final DigitalInput bottom = new DigitalInput(0);

    private final double coralElevatorInitialSetpoint = 6.375;
    private final double coralWristSetpoint = -84.0;
    private final double coralArmSetpoint = 115.5;
    //private final double coralElevatorSetpoint = 18.3; // For bad coral station
    private final double coralElevatorSetpoint = 17.05;

    private final double floorElevatorSetpoint = 0.9;
    private final double floorArmSetpoint = -35.0;
    private final double floorWristSetpoint = 33.5;

    private final double l1ElevatorSetpoint = 1.7;
    private final double l1ArmSetpoint = 115.0;
    private final double l1WristSetpoint = -80.0;

    private final double l2ArmSetpoint = 69.2;
    private final double l2WristSetpoint = 55.0;
    private final double l2ElevatorSetpoint = 9.2;

    private final double l3ArmSetpoint = 69.2;
    private final double l3WristSetpoint = 55.0;
    private final double l3ElevatorSetpoint = 23.5;

    private final double l4ArmSetpoint = 69.2;
    private final double l4WristSetpoint = 50.0;
    private final double l4ElevatorSetpoint = 44.5;

    private final double algaeLowArmSetpoint = 40.0;
    private final double algaeLowWristSetpoint = 10.0;
    private final double algaeLowElevatorSetpoint = 6.2;

    private final double algaeHighArmSetpoint = 40.0;
    private final double algaeHighWristSetpoint = 10.0;
    private final double algaeHighElevatorSetpoint = 20.0;

    private final double algaeScoreLowArmSetpoint = 56.0;
    private final double algaeScoreLowWristSetpoint = 74.0;
    private final double algaeScoreLowElevatorSetpoint = 7.7;

    private final double algaeScoreHighArmSetpoint = 56.0;
    private final double algaeScoreHighWristSetpoint = 74.0;
    private final double algaeScoreHighElevatorSetpoint = 21.5;

    private final double elevatorRotationsToInches = 0.4327;

    private boolean elevatorOverride = false;
    private double elevatorOverrideValue = 0.0;
    private boolean armOverride = false;
    private double armOverrideValue = 0.0;
    private boolean wristOverride = false;
    private double wristOverrideValue = 0.0;

    public Elevator() {
        elevator.getConfigurator().apply(outputConfigs);
        elevator.getConfigurator().apply(voltageConfigs);
        elevator.getConfigurator().apply(elevatorCurrentConfigs);
        elevator.getConfigurator().apply(elevatorPIDConfigs);
        elevatorFollow.getConfigurator().apply(outputConfigs);
        elevatorFollow.getConfigurator().apply(voltageConfigs);
        elevatorFollow.getConfigurator().apply(elevatorCurrentConfigs);
        elevatorFollow.getConfigurator().apply(elevatorPIDConfigs);
        arm.getConfigurator().apply(outputConfigs);
        arm.getConfigurator().apply(armVoltageConfigs);
        arm.getConfigurator().apply(currentConfigs);
        arm.getConfigurator().apply(armFeedbackConfigs);
        arm.getConfigurator().apply(armPIDConfigs);
        wrist.getConfigurator().apply(outputConfigs);
        wrist.getConfigurator().apply(wristVoltageConfigs);
        wrist.getConfigurator().apply(currentConfigs);
        wrist.getConfigurator().apply(wristFeedbackConfigs);
        wrist.getConfigurator().apply(wristPIDConfigs);
        armEncoder.getConfigurator().apply(armEncoderConfigs);
        wristEncoder.getConfigurator().apply(wristEncoderConfigs);

        elevatorPosition = elevator.getPosition();
        elevatorVelocity = elevator.getVelocity();
        elevatorVoltage = elevator.getMotorVoltage();
        elevatorTemp = elevator.getDeviceTemp();
        elevatorPositionFollow = elevatorFollow.getPosition();
        elevatorVelocityFollow = elevatorFollow.getVelocity();
        elevatorVoltageFollow = elevatorFollow.getMotorVoltage();
        armPosition = arm.getPosition();
        armVelocity = arm.getVelocity();
        armVoltage = arm.getMotorVoltage();
        wristPosition = wrist.getPosition();
        wristVelocity = wrist.getVelocity();
        wristVoltage = wrist.getMotorVoltage();
        armEncoderPosition = armEncoder.getAbsolutePosition();
        armEncoderVelocity = armEncoder.getVelocity();
        wristEncoderPosition = wristEncoder.getAbsolutePosition();
        wristEncoderVelocity = wristEncoder.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(50,
            elevatorPosition, elevatorVelocity, elevatorVoltage, elevatorTemp, elevatorPositionFollow,
            elevatorVelocityFollow, elevatorVoltageFollow, armPosition, armVelocity, armVoltage,
            wristPosition, wristVelocity, wristVoltage, armEncoderPosition, armEncoderVelocity,
            wristEncoderPosition, wristEncoderVelocity);
        ParentDevice.optimizeBusUtilizationForAll(
            elevator, elevatorFollow, arm, wrist, armEncoder, wristEncoder);

        elevatorPosition.waitForUpdate(0.02);
        elevator.setPosition(Rotations.of(0.0));

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
        return elevatorRotationsToInches(elevatorPosition());
    }

    public double elevatorPositionFollow() {
        return elevatorPositionFollow.getValue().in(Rotations);
    }

    public double elevatorHeightFollow() {
        return elevatorRotationsToInches(elevatorPositionFollow());
    }

    public double elevatorVelocity() {
        return elevatorVelocity.getValue().in(RotationsPerSecond);
    }

    public double elevatorVelocityFollow() {
        return elevatorVelocityFollow.getValue().in(RotationsPerSecond);
    }

    public double elevatorVoltage() {
        return elevatorVoltage.getValue().in(Volts);
    }

    public double elevatorVoltageFollow() {
        return elevatorVoltageFollow.getValue().in(Volts);
    }

    public double elevatorTemp() {
        return elevatorTemp.getValue().in(Fahrenheit);
    }

    public double armPosition() {
        return armPosition.getValue().in(Degrees);
    }

    public double armVelocity() {
        return armVelocity.getValue().in(DegreesPerSecond);
    }

    public double armVoltage() {
        return armVoltage.getValue().in(Volts);
    }

    public double wristPosition() {
        return wristPosition.getValue().in(Degrees);
    }

    public double wristVelocity() {
        return wristVelocity.getValue().in(DegreesPerSecond);
    }

    public double wristVoltage() {
        return wristVoltage.getValue().in(Volts);
    }

    public double armEncoderPosition() {
        return armEncoderPosition.getValue().in(Degrees);
    }

    public double elevatorRotationsToInches(double rotations) {
        return rotations * elevatorRotationsToInches;
    }

    public double elevatorInchesToRotations(double inches) {
        return inches / elevatorRotationsToInches;
    }

    public boolean bottomLimit() {
        return bottom.get();
    }

    public boolean elevatorAtSetpoint(double setpoint) {
        return Math.abs(elevatorPosition() - setpoint) < 1.0;
    }

    public boolean armAtSetpoint(double setpoint) {
        return Math.abs(armPosition() - setpoint) < 5.0;
    }

    public boolean armAtSetpointLoose(double setpoint) {
        return Math.abs(armPosition() - setpoint) < 90.0;
    }

    public boolean wristAtSetpoint(double setpoint) {
        return Math.abs(wristPosition() - setpoint) < 5.0;
    }

    public boolean wristAtSetpointLoose(double setpoint) {
        return Math.abs(wristPosition() - setpoint) < 90.0;
    }

    public Command resetElevator() {
        return runOnce(() -> elevator.setPosition(0));
    }

    public Command moveElevatorUp() {
        return runOnce(() -> elevator.setControl(
            positionRequest.withPosition(elevatorPosition.getValue().plus(Rotations.of(elevatorInchesToRotations(0.125))))
        ));
    }

    public Command moveElevatorDown() {
        return runOnce(() -> elevator.setControl(
            positionRequest.withPosition(elevatorPosition.getValue().minus(Rotations.of(elevatorInchesToRotations(0.125))))
        ));
    }

    public Command moveToCoralStation() {
        return Commands.sequence(
            run(() -> elevator.setControl(
                positionRequest.withPosition(Rotations.of(coralElevatorInitialSetpoint))
            )).until(() -> elevatorAtSetpoint(coralElevatorInitialSetpoint)),
            run(() -> {
                arm.setControl(
                    positionRequest.withPosition(Degrees.of(coralArmSetpoint))
                );
                wrist.setControl(
                    positionRequest.withPosition(Degrees.of(coralWristSetpoint))
                );
            }).until(() -> armAtSetpoint(coralArmSetpoint) && wristAtSetpoint(coralWristSetpoint)),
            run(() -> elevator.setControl(
                positionRequest.withPosition(Rotations.of(coralElevatorSetpoint))
            )).until(() -> elevatorAtSetpoint(coralElevatorSetpoint))
        );
    }

    public Command moveToFloorPickup() {
        return Commands.sequence(
            run(() -> elevator.setControl(
                positionRequest.withPosition(Rotations.of(floorElevatorSetpoint))
            )).until(() -> elevatorAtSetpoint(floorElevatorSetpoint)),
            run(() -> {
                arm.setControl(
                    positionRequest.withPosition(Degrees.of(floorArmSetpoint))
                );
                wrist.setControl(
                    positionRequest.withPosition(Degrees.of(floorWristSetpoint))
                );
            }).until(() -> armAtSetpoint(floorArmSetpoint) && wristAtSetpoint(floorWristSetpoint))
        );
    }

    public Command moveToL1() {
        return Commands.either(
            Commands.sequence(
                run(() -> elevator.setControl(
                    positionRequest.withPosition(Rotations.of(l1ElevatorSetpoint))
                )).until(() -> elevatorAtSetpoint(l1ElevatorSetpoint)),
                run(() -> {
                    arm.setControl(
                        positionRequest.withPosition(Degrees.of(l1ArmSetpoint))
                    );
                }).until(() -> armAtSetpointLoose(l1ArmSetpoint)),
                run(() -> {
                    wrist.setControl(
                        positionRequest.withPosition(Degrees.of(l1WristSetpoint))
                    );
                }).until(() -> wristAtSetpoint(l1WristSetpoint))
            ), 
            Commands.sequence(
                run(() -> elevator.setControl(
                    positionRequest.withPosition(Rotations.of(l1ElevatorSetpoint))
                )).until(() -> elevatorAtSetpoint(l1ElevatorSetpoint)),
                run(() -> {
                    arm.setControl(
                        positionRequest.withPosition(Degrees.of(l1ArmSetpoint))
                    );
                    wrist.setControl(
                        positionRequest.withPosition(Degrees.of(l1WristSetpoint))
                    );
                }).until(() -> armAtSetpoint(l1ArmSetpoint) && wristAtSetpoint(l1WristSetpoint))
            ), 
            () -> elevatorPosition() < 2.0);
    }

    public Command moveToL2() {
        return Commands.sequence(
            run(() -> {
                arm.setControl(
                    positionRequest.withPosition(Degrees.of(l2ArmSetpoint))
                );
                wrist.setControl(
                    positionRequest.withPosition(Degrees.of(l2WristSetpoint))
                );
            }).until(() -> armAtSetpoint(l2ArmSetpoint) && wristAtSetpoint(l2WristSetpoint)),
            run(() -> elevator.setControl(
                positionRequest.withPosition(Rotations.of(l2ElevatorSetpoint))
            )).until(() -> elevatorAtSetpoint(l2ElevatorSetpoint))
        );
    }

    public Command moveToL3() {
        return Commands.sequence(
            run(() -> {
                arm.setControl(
                    positionRequest.withPosition(Degrees.of(l3ArmSetpoint))
                );
                wrist.setControl(
                    positionRequest.withPosition(Degrees.of(l3WristSetpoint))
                );
            }).until(() -> armAtSetpoint(l3ArmSetpoint) && wristAtSetpoint(l3WristSetpoint)),
            run(() -> elevator.setControl(
                positionRequest.withPosition(Rotations.of(l3ElevatorSetpoint))
            )).until(() -> elevatorAtSetpoint(l3ElevatorSetpoint))
        );
    }

    public Command moveToL4() {
        return Commands.sequence(
            run(() -> {
                arm.setControl(
                    positionRequest.withPosition(Degrees.of(l4ArmSetpoint))
                );
                wrist.setControl(
                    positionRequest.withPosition(Degrees.of(l4WristSetpoint))
                );
            }).until(() -> armAtSetpoint(l4ArmSetpoint) && wristAtSetpoint(l4WristSetpoint)),
            run(() -> elevator.setControl(
                positionRequest.withPosition(Rotations.of(l4ElevatorSetpoint))
            )).until(() -> elevatorAtSetpoint(l4ElevatorSetpoint))
        );
    }

    public Command moveToAlgaeLow() {
        return Commands.sequence(
            run(() -> {
                arm.setControl(
                    positionRequest.withPosition(Degrees.of(algaeLowArmSetpoint))
                );
                wrist.setControl(
                    positionRequest.withPosition(Degrees.of(algaeLowWristSetpoint))
                );
            }).until(() -> armAtSetpoint(algaeLowArmSetpoint) && wristAtSetpoint(algaeLowWristSetpoint)),
            run(() -> elevator.setControl(
                positionRequest.withPosition(Rotations.of(algaeLowElevatorSetpoint))
            )).until(() -> elevatorAtSetpoint(algaeLowElevatorSetpoint))
        );
    }

    public Command moveToAlgaeHigh() {
        return Commands.sequence(
            run(() -> {
                arm.setControl(
                    positionRequest.withPosition(Degrees.of(algaeHighArmSetpoint))
                );
                wrist.setControl(
                    positionRequest.withPosition(Degrees.of(algaeHighWristSetpoint))
                );
            }).until(() -> armAtSetpoint(algaeHighArmSetpoint) && wristAtSetpoint(algaeHighWristSetpoint)),
            run(() -> elevator.setControl(
                positionRequest.withPosition(Rotations.of(algaeHighElevatorSetpoint))
            )).until(() -> elevatorAtSetpoint(algaeHighElevatorSetpoint))
        );
    }

    public Command moveToAlgaeScoreLow() {
        return Commands.sequence(
            run(() -> {
                arm.setControl(
                    positionRequest.withPosition(Degrees.of(algaeScoreLowArmSetpoint))
                );
                wrist.setControl(
                    positionRequest.withPosition(Degrees.of(algaeScoreLowWristSetpoint))
                );
            }).until(() -> armAtSetpoint(algaeScoreLowArmSetpoint) && wristAtSetpoint(algaeScoreLowWristSetpoint)),
            run(() -> elevator.setControl(
                positionRequest.withPosition(Rotations.of(algaeScoreLowElevatorSetpoint))
            )).until(() -> elevatorAtSetpoint(algaeScoreLowElevatorSetpoint))
        );
    }

    public Command moveToAlgaeScoreHigh() {
        return Commands.sequence(
            run(() -> {
                arm.setControl(
                    positionRequest.withPosition(Degrees.of(algaeScoreHighArmSetpoint))
                );
                wrist.setControl(
                    positionRequest.withPosition(Degrees.of(algaeScoreHighWristSetpoint))
                );
            }).until(() -> armAtSetpoint(algaeScoreHighArmSetpoint) && wristAtSetpoint(algaeScoreHighWristSetpoint)),
            run(() -> elevator.setControl(
                positionRequest.withPosition(Rotations.of(algaeScoreHighElevatorSetpoint))
            )).until(() -> elevatorAtSetpoint(algaeScoreHighElevatorSetpoint))
        );
    }

    public Command sysIdElevatorQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdElevator.quasistatic(direction);
    }

    public Command sysIdElevatorDynamic(SysIdRoutine.Direction direction) {
        return sysIdElevator.dynamic(direction);
    }

    public Command sysIdArmQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdArm.quasistatic(direction);
    }

    public Command sysIdArmDynamic(SysIdRoutine.Direction direction) {
        return sysIdArm.dynamic(direction);
    }

    public Command sysIdWristQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdWrist.quasistatic(direction);
    }

    public Command sysIdWristDynamic(SysIdRoutine.Direction direction) {
        return sysIdWrist.dynamic(direction);
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
            elevatorPosition, elevatorVelocity, elevatorVoltage, elevatorTemp, elevatorPositionFollow,
            elevatorVelocityFollow, elevatorVoltageFollow, armPosition, armVelocity, armVoltage,
            wristPosition, wristVelocity, wristVoltage, armEncoderPosition, armEncoderVelocity,
            wristEncoderPosition, wristEncoderVelocity);

        if (SmartDashboard.getBoolean("ElevatorOverride", elevatorOverride)) {
            elevator.setControl(positionRequest.withPosition(Rotations.of(elevatorInchesToRotations(
                SmartDashboard.getNumber("ElevatorOverrideValue", elevatorOverrideValue)))));
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
