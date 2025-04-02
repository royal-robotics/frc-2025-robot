package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.Vision.PoseEstimate;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
@Logged
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    private final Vision vision = new Vision();
    private final Field2d field = new Field2d();

    private int closestReefTag = 0;
    private Pose2d closestLeftPole = new Pose2d();
    private Pose2d closestRightPole = new Pose2d();

    private int closestCoralStationTag = 0;
    private Pose2d closestFarStation = new Pose2d();
    private Pose2d closestNearStation = new Pose2d();

    private final ProfiledPIDController xController = new ProfiledPIDController(
        8.0, 0.0, 0.0, new Constraints(1.0, 3.0));
    private final ProfiledPIDController yController = new ProfiledPIDController(
        8.0, 0.0, 0.0, new Constraints(1.0, 3.0));

    // Red REEF Positions
    private final Pose2d sixLeft = new Pose2d(13.574, 2.809, new Rotation2d(Degrees.of(300)));
    private final Pose2d sixRight = new Pose2d(13.862, 2.964, new Rotation2d(Degrees.of(300)));
    private final Pose2d sevenLeft = new Pose2d(14.559, 3.861, new Rotation2d(Degrees.of(0)));
    private final Pose2d sevenRight = new Pose2d(14.559, 4.191, new Rotation2d(Degrees.of(0)));
    private final Pose2d eightLeft = new Pose2d(13.862, 5.088, new Rotation2d(Degrees.of(60)));
    private final Pose2d eightRight = new Pose2d(13.574, 5.243, new Rotation2d(Degrees.of(60)));
    private final Pose2d nineLeft = new Pose2d(12.546, 5.243, new Rotation2d(Degrees.of(120)));
    private final Pose2d nineRight = new Pose2d(12.259, 5.088, new Rotation2d(Degrees.of(120)));
    private final Pose2d tenLeft = new Pose2d(11.559, 4.191, new Rotation2d(Degrees.of(180)));
    private final Pose2d tenRight = new Pose2d(11.559, 3.861, new Rotation2d(Degrees.of(180)));
    private final Pose2d elevenLeft = new Pose2d(12.259, 2.964, new Rotation2d(Degrees.of(240)));
    private final Pose2d elevenRight = new Pose2d(12.546, 2.809, new Rotation2d(Degrees.of(240)));

    // Blue REEF Positions
    private final Pose2d seventeenLeft = new Pose2d(3.689, 2.964, new Rotation2d(Degrees.of(60)));
    private final Pose2d seventeenRight = new Pose2d(3.973, 2.809, new Rotation2d(Degrees.of(60)));
    private final Pose2d eighteenLeft = new Pose2d(3.169, 4.191, new Rotation2d(Degrees.of(0)));
    private final Pose2d eighteenRight = new Pose2d(3.169, 3.861, new Rotation2d(Degrees.of(0)));
    private final Pose2d nineteenLeft = new Pose2d(3.973, 5.243, new Rotation2d(Degrees.of(300)));
    private final Pose2d nineteenRight = new Pose2d(3.689, 5.088, new Rotation2d(Degrees.of(300)));
    private final Pose2d twentyLeft = new Pose2d(5.289, 5.088, new Rotation2d(Degrees.of(240)));
    private final Pose2d twentyRight = new Pose2d(5.014, 5.243, new Rotation2d(Degrees.of(240)));
    private final Pose2d twentyOneLeft = new Pose2d(5.809, 3.861, new Rotation2d(Degrees.of(180)));
    private final Pose2d twentyOneRight = new Pose2d(5.809, 4.191, new Rotation2d(Degrees.of(180)));
    private final Pose2d twentyTwoLeft = new Pose2d(5.014, 2.809, new Rotation2d(Degrees.of(120)));
    private final Pose2d twentyTwoRight = new Pose2d(5.289, 2.964, new Rotation2d(Degrees.of(120)));

    // Red CORAL STATION Positions
    private final Pose2d oneFar = new Pose2d(15.92, 0.65, new Rotation2d(Degrees.of(125)));
    private final Pose2d oneNear = new Pose2d(16.95, 1.39, new Rotation2d(Degrees.of(125)));
    private final Pose2d twoFar = new Pose2d(15.95, 7.47, new Rotation2d(Degrees.of(235)));
    private final Pose2d twoNear = new Pose2d(16.89, 6.70, new Rotation2d(Degrees.of(235)));

    // Blue CORAL STATION Positions
    private final Pose2d twelveFar = new Pose2d(1.64, 0.65, new Rotation2d(Degrees.of(235)));
    private final Pose2d twelveNear = new Pose2d(0.65, 1.42, new Rotation2d(Degrees.of(235)));
    private final Pose2d thirteenFar = new Pose2d(1.65, 7.47, new Rotation2d(Degrees.of(125)));
    private final Pose2d thirteenNear = new Pose2d(0.64, 6.70, new Rotation2d(Degrees.of(125)));

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    private final SwerveRequest.FieldCentricFacingAngle m_driveToPoint = new SwerveRequest.FieldCentricFacingAngle()
        .withHeadingPID(8.0, 0.0, 0.0);

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command driveToReefPoint(boolean left) {
        return applyRequest(() -> {
            Pose2d newPose = getState().Pose;
            if (left) {
                if (newPose.getX() > 8.75) {
                    return m_driveToPoint
                    .withVelocityX(-xController.calculate(newPose.getX()))
                    .withVelocityY(-yController.calculate(newPose.getY()))
                    .withTargetDirection(closestLeftPole.getRotation());
                } else {
                    return m_driveToPoint
                    .withVelocityX(xController.calculate(newPose.getX()))
                    .withVelocityY(yController.calculate(newPose.getY()))
                    .withTargetDirection(closestLeftPole.getRotation());
                }
            } else {
                if (newPose.getX() > 8.75) {
                    return m_driveToPoint
                    .withVelocityX(-xController.calculate(newPose.getX()))
                    .withVelocityY(-yController.calculate(newPose.getY()))
                    .withTargetDirection(closestRightPole.getRotation());
                } else {
                    return m_driveToPoint
                    .withVelocityX(xController.calculate(newPose.getX()))
                    .withVelocityY(yController.calculate(newPose.getY()))
                    .withTargetDirection(closestRightPole.getRotation());
                }
            }
        }).beforeStarting(runOnce(() -> {
            Pose2d initialPose = getState().Pose;

            xController.reset(initialPose.getX());
            yController.reset(initialPose.getY());

            if (left) {
                xController.setGoal(closestLeftPole.getX());
                yController.setGoal(closestLeftPole.getY());
            } else {
                xController.setGoal(closestRightPole.getX());
                yController.setGoal(closestRightPole.getY());
            }
        }));
    }

    public Command driveToCoralStationPoint(boolean far) {
        return applyRequest(() -> {
            Pose2d newPose = getState().Pose;
            if (far) {
                if (newPose.getX() > 8.75) {
                    return m_driveToPoint
                    .withVelocityX(-xController.calculate(newPose.getX()))
                    .withVelocityY(-yController.calculate(newPose.getY()))
                    .withTargetDirection(closestFarStation.getRotation());
                } else {
                    return m_driveToPoint
                    .withVelocityX(xController.calculate(newPose.getX()))
                    .withVelocityY(yController.calculate(newPose.getY()))
                    .withTargetDirection(closestFarStation.getRotation());
                }
            } else {
                if (newPose.getX() > 8.75) {
                    return m_driveToPoint
                    .withVelocityX(-xController.calculate(newPose.getX()))
                    .withVelocityY(-yController.calculate(newPose.getY()))
                    .withTargetDirection(closestNearStation.getRotation());
                } else {
                    return m_driveToPoint
                    .withVelocityX(xController.calculate(newPose.getX()))
                    .withVelocityY(yController.calculate(newPose.getY()))
                    .withTargetDirection(closestNearStation.getRotation());
                }
            }
        }).beforeStarting(runOnce(() -> {
            Pose2d initialPose = getState().Pose;

            xController.reset(initialPose.getX());
            yController.reset(initialPose.getY());

            if (far) {
                xController.setGoal(closestFarStation.getX());
                yController.setGoal(closestFarStation.getY());
            } else {
                xController.setGoal(closestNearStation.getX());
                yController.setGoal(closestNearStation.getY());
            }
        }));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the translation routine
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdTranslationQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineTranslation.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the translation routine
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdTranslationDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineTranslation.dynamic(direction);
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the steer routine
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdSteerQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineSteer.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the steer routine
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdSteerDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineSteer.dynamic(direction);
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the rotation routine
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdRotationQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineRotation.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the rotation routine
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdRotationDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineRotation.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        Pose2d currentPose = getState().Pose;
        field.setRobotPose(currentPose);
        PoseEstimate visionPose = vision.getEstimatedPose();
        if (visionPose != null) {
            addVisionMeasurement(
                visionPose.getEstimatedRobotPose().estimatedPose.toPose2d(),
                visionPose.getEstimatedRobotPose().timestampSeconds,
                visionPose.getEstimatedStdDev());
            field.getObject("Vision").setPose(visionPose.getEstimatedRobotPose().estimatedPose.toPose2d());
        }

        SmartDashboard.putData(field);

        if (currentPose.getX() > 8.75) {
            if (currentPose.getX() > 13.05) {
                double distanceToSix = currentPose.getTranslation().getDistance(
                    vision.getTagPose(6).toPose2d().getTranslation()
                );
                double distanceToSeven = currentPose.getTranslation().getDistance(
                    vision.getTagPose(7).toPose2d().getTranslation()
                );
                double distanceToEight = currentPose.getTranslation().getDistance(
                    vision.getTagPose(8).toPose2d().getTranslation()
                );

                if (distanceToSix < distanceToSeven && distanceToSix < distanceToEight) {
                    closestReefTag = 6;
                    closestLeftPole = sixLeft;
                    closestRightPole = sixRight;
                } else if (distanceToSeven < distanceToEight) {
                    closestReefTag = 7;
                    closestLeftPole = sevenLeft;
                    closestRightPole = sevenRight;
                } else {
                    closestReefTag = 8;
                    closestLeftPole = eightLeft;
                    closestRightPole = eightRight;
                }
            } else {
                double distanceToNine = currentPose.getTranslation().getDistance(
                    vision.getTagPose(9).toPose2d().getTranslation()
                );
                double distanceToTen = currentPose.getTranslation().getDistance(
                    vision.getTagPose(10).toPose2d().getTranslation()
                );
                double distanceToEleven = currentPose.getTranslation().getDistance(
                    vision.getTagPose(11).toPose2d().getTranslation()
                );

                if (distanceToNine < distanceToTen && distanceToNine < distanceToEleven) {
                    closestReefTag = 9;
                    closestLeftPole = nineLeft;
                    closestRightPole = nineRight;
                } else if (distanceToTen < distanceToEleven) {
                    closestReefTag = 10;
                    closestLeftPole = tenLeft;
                    closestRightPole = tenRight;
                } else {
                    closestReefTag = 11;
                    closestLeftPole = elevenLeft;
                    closestRightPole = elevenRight;
                }
            }

            if (currentPose.getY() < 4.0) {
                closestCoralStationTag = 1;
                closestFarStation = oneFar;
                closestNearStation = oneNear;
            } else {
                closestCoralStationTag = 2;
                closestFarStation = twoFar;
                closestNearStation = twoNear;
            }
        } else {
            if (currentPose.getX() < 4.49) {
                double distanceToSeventeen = currentPose.getTranslation().getDistance(
                    vision.getTagPose(17).toPose2d().getTranslation()
                );
                double distanceToEighteen = currentPose.getTranslation().getDistance(
                    vision.getTagPose(18).toPose2d().getTranslation()
                );
                double distanceToNineteen = currentPose.getTranslation().getDistance(
                    vision.getTagPose(19).toPose2d().getTranslation()
                );

                if (distanceToSeventeen < distanceToEighteen && distanceToSeventeen < distanceToNineteen) {
                    closestReefTag = 17;
                    closestLeftPole = seventeenLeft;
                    closestRightPole = seventeenRight;
                } else if (distanceToEighteen < distanceToNineteen) {
                    closestReefTag = 18;
                    closestLeftPole = eighteenLeft;
                    closestRightPole = eighteenRight;
                } else {
                    closestReefTag = 19;
                    closestLeftPole = nineteenLeft;
                    closestRightPole = nineteenRight;
                }
            } else {
                double distanceToTwenty = currentPose.getTranslation().getDistance(
                    vision.getTagPose(20).toPose2d().getTranslation()
                );
                double distanceToTwentyOne = currentPose.getTranslation().getDistance(
                    vision.getTagPose(21).toPose2d().getTranslation()
                );
                double distanceToTwentyTwo = currentPose.getTranslation().getDistance(
                    vision.getTagPose(22).toPose2d().getTranslation()
                );

                if (distanceToTwenty < distanceToTwentyOne && distanceToTwenty < distanceToTwentyTwo) {
                    closestReefTag = 20;
                    closestLeftPole = twentyLeft;
                    closestRightPole = twentyRight;
                } else if (distanceToTwentyOne < distanceToTwentyTwo) {
                    closestReefTag = 21;
                    closestLeftPole = twentyOneLeft;
                    closestRightPole = twentyOneRight;
                } else {
                    closestReefTag = 22;
                    closestLeftPole = twentyTwoLeft;
                    closestRightPole = twentyTwoRight;
                }
            }

            if (currentPose.getY() < 4.0) {
                closestCoralStationTag = 12;
                closestFarStation = twelveFar;
                closestNearStation = twelveNear;
            } else {
                closestCoralStationTag = 13;
                closestFarStation = thirteenFar;
                closestNearStation = thirteenNear;
            }
        }

        SmartDashboard.putNumber("Closest Reef Tag", closestReefTag);
        SmartDashboard.putNumber("Closes Station Tag", closestCoralStationTag);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }
}

