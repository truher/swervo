package frc.math;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

/**
 * Some simple vector operations useful for tracking.
 */
public abstract class Vectors {

    public static <R extends Num, C extends Num> double norm(Matrix<R, C> v) {
        return Math.sqrt(v.transpose().times(v).get(0, 0));
    }

    public static Vector<N2> translationVectorFromTranslation(Translation2d translation) {
        return VecBuilder.fill(translation.getX(), translation.getY());
    }

    public static Vector<N2> translationVectorFromPose(Pose2d pose) {
        return translationVectorFromTranslation(pose.getTranslation());
    }

    public static Vector<N2> velocityVectorFromPoseAndSpeed(Pose2d pose, double speed) {
        return VecBuilder.fill(
                speed * pose.getRotation().getCos(),
                speed * pose.getRotation().getSin());
    }

    public static Matrix<N2, N1> targetRelative(Translation2d aimingPoint, Pose2d pose) {
        Vector<N2> robotPosition = translationVectorFromPose(pose);
        Vector<N2> targetPosition = translationVectorFromTranslation(aimingPoint);
        return targetPosition.minus(robotPosition);
    }

    public static Rotation2d targetRelativeRotation(Translation2d aimingPoint, Pose2d pose) {
        Matrix<N2, N1> targetRelative = targetRelative(aimingPoint, pose);
        return new Rotation2d(targetRelative.get(0, 0), targetRelative.get(1, 0));
    }

    public static double tangentialSpeedMetersPerSec(
            Translation2d aimingPoint,
            Pose2d pose,
            double speed) {
        Matrix<N2, N1> targetRelative = targetRelative(aimingPoint, pose);
        Matrix<N2, N1> targetDirection = targetRelative.div(norm(targetRelative));
        Vector<N2> robotVelocity = velocityVectorFromPoseAndSpeed(pose, speed);
        double radialComponent = robotVelocity.transpose().times(targetDirection).get(0, 0);
        Matrix<N2, N1> radialVelocity = targetDirection.times(radialComponent);
        Matrix<N2, N1> tangentialVelocity = robotVelocity.minus(radialVelocity);
        return norm(tangentialVelocity);
    }
}
