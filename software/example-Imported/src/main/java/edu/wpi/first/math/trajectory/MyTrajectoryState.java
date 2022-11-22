package edu.wpi.first.math.trajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory.State;

/**
 * Allows interpolation of pure rotations.
 */
public class MyTrajectoryState extends Trajectory.State {

    private static double lerp(double startValue, double endValue, double t) {
        return startValue + (endValue - startValue) * t;
    }

    private static Pose2d lerp(Pose2d startValue, Pose2d endValue, double t) {
        return startValue.plus((endValue.minus(startValue)).times(t));
    }

    public MyTrajectoryState(double timeSeconds, double velocityMetersPerSecond, double accelerationMetersPerSecondSq,
            Pose2d poseMeters, double curvatureRadPerMeter) {
        super(timeSeconds, velocityMetersPerSecond, accelerationMetersPerSecondSq, poseMeters, curvatureRadPerMeter);
    }

    /**
     * just interpolate, don't bother with trajectory-timing blah blah
     */
    State interpolate(State endValue, double i) {
        return new MyTrajectoryState(
                lerp(timeSeconds, endValue.timeSeconds, i),
                lerp(velocityMetersPerSecond, endValue.velocityMetersPerSecond, i),
                accelerationMetersPerSecondSq,
                lerp(poseMeters, endValue.poseMeters, i),
                lerp(curvatureRadPerMeter, endValue.curvatureRadPerMeter, i));
    }

    /**
     * time interpolation
     */
   // @Override
    State interpolate2(State endValue, double i) {
        final double newT = lerp(timeSeconds, endValue.timeSeconds, i);
        final double deltaT = newT - timeSeconds;
        final double newV = velocityMetersPerSecond + (accelerationMetersPerSecondSq * deltaT);
        final double newS = (velocityMetersPerSecond * deltaT
                + 0.5 * accelerationMetersPerSecondSq * Math.pow(deltaT, 2));
        // wpi says interpolation is along a moving trajectory
        // but what about stationary trajectory with pure rotation?
        // this uses velocity and acceleration to come up with pose and curvature
        // i guess pose could just vary linearly since there's no theta V or A anywhere
        final double interpolationFrac = newS
                / endValue.poseMeters.getTranslation().getDistance(poseMeters.getTranslation());
        Pose2d interpolatedPose = (interpolationFrac > 1e-2) ? //
                lerp(poseMeters, endValue.poseMeters, interpolationFrac)
                : lerp(poseMeters, endValue.poseMeters, i);
        return new MyTrajectoryState(
                newT,
                newV,
                accelerationMetersPerSecondSq,
                interpolatedPose,
                lerp(curvatureRadPerMeter, endValue.curvatureRadPerMeter, interpolationFrac));
    }

}
