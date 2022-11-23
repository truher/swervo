import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.spline.SplineHelper;
import edu.wpi.first.math.trajectory.MyTrajectoryState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

public class TrajectoryTest {
        public static final double DELTA = 0.02;

        /**
         * hunt down the "malformed spline" exception
         */
        @Test
        public void testTrajectory() {
                final double kMaxSpeedMetersPerSecond = 0.5;
                final double kMaxAccelerationMetersPerSecondSquared = 0.5;
                final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                                new Translation2d(0.1613, 0), // front
                                new Translation2d(-0.0807, 0.1397), // left rear
                                new Translation2d(-0.0807, -0.1397)); // right rear

                final TrajectoryConfig kTrajectoryConfig = new TrajectoryConfig(
                                kMaxSpeedMetersPerSecond,
                                kMaxAccelerationMetersPerSecondSquared)
                                                .setKinematics(kDriveKinematics);

                TrajectoryGenerator.setErrorHandler((String s, StackTraceElement[] t) -> {
                        throw new RuntimeException(s);
                });

                Trajectory kExampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                List.of(
                                                new Pose2d(0, 0, new Rotation2d(1, 1)),
                                                new Pose2d(0.2, 0, new Rotation2d(-1, 1)),
                                                new Pose2d(0.2, 0.2, new Rotation2d(-1, -1)),
                                                new Pose2d(0, 0.2, new Rotation2d(1, -1)),
                                                new Pose2d(0, 0, new Rotation2d(1, 1)) //
                                ),
                                kTrajectoryConfig);
                assertEquals(9.14, kExampleTrajectory.getTotalTimeSeconds(), DELTA);
        }

        // @Test
        public void testSpin() {
                final double kMaxSpeedMetersPerSecond = 0.5;
                final double kMaxAccelerationMetersPerSecondSquared = 0.5;
                final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                                new Translation2d(0.1613, 0), // front
                                new Translation2d(-0.0807, 0.1397), // left rear
                                new Translation2d(-0.0807, -0.1397)); // right rear

                final TrajectoryConfig kTrajectoryConfig = new TrajectoryConfig(
                                kMaxSpeedMetersPerSecond,
                                kMaxAccelerationMetersPerSecondSquared)
                                                .setKinematics(kDriveKinematics);

                TrajectoryGenerator.setErrorHandler((String s, StackTraceElement[] t) -> {
                        throw new RuntimeException(s);
                });

                List<Pose2d> waypoints = List.of(
                                new Pose2d(0, 0, new Rotation2d(1, 1)),
                                new Pose2d(0, 0, new Rotation2d(0, 1)),
                                new Pose2d(0, 0, new Rotation2d(-1, 1)),
                                new Pose2d(0, 0, new Rotation2d(-1, 0)),
                                new Pose2d(0, 0, new Rotation2d(-1, -1)),
                                new Pose2d(0, 0, new Rotation2d(0, -1)),
                                new Pose2d(0, 0, new Rotation2d(1, -1)),
                                new Pose2d(0, 0, new Rotation2d(1, 0)),
                                new Pose2d(0, 0, new Rotation2d(1, 1)));

                Spline[] splines = SplineHelper.getQuinticSplinesFromWaypoints(waypoints);
                assertEquals(8, splines.length);
                assertEquals(0, splines[0].getPoint(0).poseMeters.getX(), DELTA);
                assertEquals(0, splines[0].getPoint(0).poseMeters.getY(), DELTA);
                assertEquals(0, splines[0].getPoint(0).poseMeters.getRotation().getRadians(), DELTA);
                assertEquals(0, splines[1].getPoint(1).poseMeters.getX(), DELTA);
                assertEquals(0, splines[1].getPoint(1).poseMeters.getY(), DELTA);
                assertEquals(0, splines[1].getPoint(1).poseMeters.getRotation().getRadians(), DELTA);

                Trajectory kExampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                waypoints,
                                kTrajectoryConfig);
                assertEquals(9.14, kExampleTrajectory.getTotalTimeSeconds(), DELTA);
        }

        /**
         * trajectory.state interpolation is broken
         * 
         * is this fixed now?
         */
        @Test
        public void testManual() {
                Trajectory trajectory = new Trajectory(
                                List.of(
                                                new Trajectory.State(0, 0, 0, new Pose2d(0, 0, new Rotation2d(1, 0)),
                                                                0),
                                                new Trajectory.State(1, 0, 0, new Pose2d(0, 0, new Rotation2d(1, 1)),
                                                                0),
                                                new Trajectory.State(2, 0, 0, new Pose2d(0, 0, new Rotation2d(0, 1)), 0)//
                                ));
                assertEquals(2, trajectory.getTotalTimeSeconds(), DELTA);
                assertEquals(0, trajectory.sample(0).poseMeters.getRotation().getRadians(), DELTA);
                assertEquals(0, trajectory.sample(0.5).poseMeters.getRotation().getRadians(), DELTA);
                assertEquals(0, trajectory.sample(1).poseMeters.getRotation().getRadians(), DELTA);
                // is this fixed now?  used to be zero
                // TODO: figure it out
                assertEquals(0.78, trajectory.sample(1.5).poseMeters.getRotation().getRadians(), DELTA);
                assertEquals(Math.PI / 2, trajectory.sample(2).poseMeters.getRotation().getRadians(), DELTA);
        }

        @Test
        public void testMyState() {
                Trajectory trajectory = new Trajectory(
                                List.of(
                                                new MyTrajectoryState(0, 0, 0, new Pose2d(0, 0, new Rotation2d(1, 0)),
                                                                0),
                                                new MyTrajectoryState(1, 0, 0, new Pose2d(0, 0, new Rotation2d(1, 1)),
                                                                0),
                                                new MyTrajectoryState(2, 0, 0, new Pose2d(0, 0, new Rotation2d(0, 1)),
                                                                0)//
                                ));
                assertEquals(2, trajectory.getTotalTimeSeconds(), DELTA);
                assertEquals(0, trajectory.sample(0).poseMeters.getRotation().getRadians(), DELTA);
                assertEquals(Math.PI / 8, trajectory.sample(0.5).poseMeters.getRotation().getRadians(), DELTA);
                assertEquals(Math.PI / 4, trajectory.sample(1).poseMeters.getRotation().getRadians(), DELTA);
                assertEquals(Math.PI * 3 / 8, trajectory.sample(1.5).poseMeters.getRotation().getRadians(), DELTA);
                assertEquals(Math.PI / 2, trajectory.sample(2).poseMeters.getRotation().getRadians(), DELTA);
        }

        @Test
        public void testPoseAddition() {
                Pose2d a = new Pose2d(0, 0, new Rotation2d(Math.PI - 0.01));
                Pose2d b = new Pose2d(0, 0, new Rotation2d(-Math.PI + 0.01));
                Transform2d c = b.minus(a);
                assertEquals(0, c.getX(), DELTA);
                assertEquals(0, c.getY(), DELTA);
                assertEquals(0.02, c.getRotation().getRadians(), DELTA);
        }
}
