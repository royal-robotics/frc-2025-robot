package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class Vision {
    private final PhotonCamera downCamera = new PhotonCamera("Down");
    private final PhotonCamera upCamera = new PhotonCamera("Up");

    private final AprilTagFieldLayout fieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    private final Transform3d downRobotToCamera = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(10.305),
            Units.inchesToMeters(11.0),
            Units.inchesToMeters(26.6315)
        ),
        new Rotation3d(
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(25.0),
            Units.degreesToRadians(0.0)
        ));
    private final Transform3d upRobotToCamera = new Transform3d(
        new Translation3d(
            Units.inchesToMeters(10.305),
            Units.inchesToMeters(-11.0),
            Units.inchesToMeters(26.6315)
        ),
        new Rotation3d(
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(25.0),
            Units.degreesToRadians(0.0)
        ));

    private final PhotonPoseEstimator downPoseEstimator;
    private final PhotonPoseEstimator upPoseEstimator;

    public Vision() {
        downPoseEstimator = new PhotonPoseEstimator(fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, downRobotToCamera);
        upPoseEstimator = new PhotonPoseEstimator(fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, upRobotToCamera);
        downPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        upPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public PoseEstimate getEstimatedPose() {
        PoseEstimate downEstimate = getEstimatedRobotPoseForCamera(downCamera, downPoseEstimator);
        PoseEstimate upEstimate = getEstimatedRobotPoseForCamera(upCamera, upPoseEstimator);

        if (downEstimate != null && upEstimate != null) {
            if (downEstimate.getEstimatedStdDev().elementSum() < upEstimate.getEstimatedStdDev().elementSum()) {
                return downEstimate;
            } else {
                return upEstimate;
            }
        } else {
            if (downEstimate != null) {
                return downEstimate;
            } else {
                return upEstimate;
            }
        }
    }

    private PoseEstimate getEstimatedRobotPoseForCamera(PhotonCamera camera, PhotonPoseEstimator poseEstimator) {
        PoseEstimate currentPoseEstimate = null;
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            if (readingIsValid(result.getTargets())) {
                Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);
                if (estimatedPose.isPresent()) {
                    EstimatedRobotPose possiblePose = estimatedPose.get();
                    if (poseIsValid(possiblePose)) {
                        currentPoseEstimate = new PoseEstimate(possiblePose, calculateStdDev(possiblePose, result.getTargets()));
                    }
                }
            }
        }

        return currentPoseEstimate;
    }

    private boolean readingIsValid(List<PhotonTrackedTarget> targets) {
        if (targets.isEmpty()) {
            return false;
        }

        for (PhotonTrackedTarget target : targets) {
            if (target.getPoseAmbiguity() > 0.2) {
                return false;
            }
        }

        return true;
    }

    private boolean poseIsValid(EstimatedRobotPose pose) {
        return pose.estimatedPose.getZ() < 0.75 &&
            pose.estimatedPose.getX() > 0.0 &&
            pose.estimatedPose.getX() < fieldLayout.getFieldLength() &&
            pose.estimatedPose.getY() > 0.0 &&
            pose.estimatedPose.getY() < fieldLayout.getFieldWidth();
    }

    private Matrix<N3, N1> calculateStdDev(EstimatedRobotPose pose, List<PhotonTrackedTarget> targets) {
        if (targets.size() == 1) {
            Optional<Pose3d> tagPose = fieldLayout.getTagPose(targets.get(0).getFiducialId());
            if (tagPose.isPresent()) {
                double distance = tagPose.get().toPose2d().getTranslation()
                    .getDistance(pose.estimatedPose.toPose2d().getTranslation());
                return VecBuilder.fill(0.5, 0.5, 1.0).times(Math.pow(distance, 2));
            } else {
                return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            }
        } else {
            double averageDistance = 0.0;
            double numTags = 0.0;
            for (PhotonTrackedTarget target : targets) {
                Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
                if (tagPose.isPresent()) {
                    averageDistance += tagPose.get().toPose2d().getTranslation()
                        .getDistance(pose.estimatedPose.toPose2d().getTranslation());
                    numTags++;
                }
            }

            if (numTags != 0) {
                averageDistance /= numTags;
                return VecBuilder.fill(0.1, 0.1, 0.2).times(averageDistance);
            } else {
                return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            }
        }
    }

    public Pose3d getTagPose(int id) {
        return fieldLayout.getTagPose(id).get();
    }

    public class PoseEstimate {
        private EstimatedRobotPose estimatedRobotPose;
        private Matrix<N3, N1> estimatedStdDev;

        public PoseEstimate(EstimatedRobotPose pose, Matrix<N3, N1> stdDev) {
            estimatedRobotPose = pose;
            estimatedStdDev = stdDev;
        }

        public EstimatedRobotPose getEstimatedRobotPose() {
            return estimatedRobotPose;
        }

        public Matrix<N3, N1> getEstimatedStdDev() {
            return estimatedStdDev;
        }


    }
}
