package frc.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;

public class TrackingConstraint implements TrajectoryConstraint {

    private final double m_maxSpeedMetersPerSecond;
    private final SwerveDriveKinematics m_kinematics;
    private final Translation2d m_aimingPoint;

    public TrackingConstraint(
            final SwerveDriveKinematics kinematics,
            double maxSpeedMetersPerSecond,
            Translation2d aimingPoint) {
        m_maxSpeedMetersPerSecond = maxSpeedMetersPerSecond;
        m_kinematics = kinematics;
        m_aimingPoint = aimingPoint;
    }

    /*
     * replace the rotation of the trajectory with a rotation aiming at a fixed
     * point.
     */
    public static Trajectory setAimingPoint(Trajectory trajectory, Translation2d aimingPoint) {
        for (Trajectory.State state : trajectory.getStates()) {
            double dx = aimingPoint.getX() - state.poseMeters.getX();
            double dy = aimingPoint.getY() - state.poseMeters.getY();
            Rotation2d rot = new Rotation2d(dx, dy);
            state.poseMeters = new Pose2d(state.poseMeters.getTranslation(), rot);
        }
        return trajectory;
    }

    /**
     * this is cribbed from SwerveDriveKinematicsConstraint.
     */
    @Override
    public double getMaxVelocityMetersPerSecond(
            Pose2d poseMeters,
            double curvatureRadPerMeter,
            double velocityMetersPerSecond) {
        // field velocities. the direction of the pose is the direction of motion of the
        // robot, not the orientation.
        var vxMetersPerSecond = velocityMetersPerSecond * poseMeters.getRotation().getCos();
        var vyMetersPerSecond = velocityMetersPerSecond * poseMeters.getRotation().getSin();

        // this is what SwerveDriveKinematicsConstraint says, and it's wrong unless
        // you're driving like a tank.
        // double omegaRadiansPerSecond = velocityMetersPerSecond *
        // curvatureRadPerMeter;

        // instead find the aiming solution (position only)
        double dxM = m_aimingPoint.getX() - poseMeters.getX();
        double dyM = m_aimingPoint.getY() - poseMeters.getY();
        double distanceToTargetM = Math.hypot(dxM, dyM);
        double tangentialSpeedMS = Vectors.tangentialSpeedMetersPerSec(
                m_aimingPoint, poseMeters, velocityMetersPerSecond);
        double omegaRadiansPerSecond = tangentialSpeedMS / distanceToTargetM;
        Rotation2d targetRelativeRotation = Vectors.targetRelativeRotation(m_aimingPoint, poseMeters);

        // this is wrong, these are field-relative speeds and angles.
        // var chassisSpeeds = new ChassisSpeeds(
        // vxMetersPerSecond,
        // vyMetersPerSecond,
        // omegaRadiansPerSecond);
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                vxMetersPerSecond,
                vyMetersPerSecond,
                omegaRadiansPerSecond,
                targetRelativeRotation);

        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, m_maxSpeedMetersPerSecond);

        ChassisSpeeds normSpeeds = m_kinematics.toChassisSpeeds(moduleStates);
        return Math.hypot(normSpeeds.vxMetersPerSecond, normSpeeds.vyMetersPerSecond);
    }

    @Override
    public MinMax getMinMaxAccelerationMetersPerSecondSq(
            Pose2d poseMeters,
            double curvatureRadPerMeter,
            double velocityMetersPerSecond) {
        return new MinMax();
    }

}
