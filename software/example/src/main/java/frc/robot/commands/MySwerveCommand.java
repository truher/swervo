package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.MyTrajectoryState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.math.TrackingConstraint;
import frc.robot.subsystems.Drivetrain;

/**
 * Autonomous trajectory. so much is wrong, and not visible, in
 * SwerveControllerCommand that i mostly copied it here.
 */
public class MySwerveCommand extends CommandBase {

    public static final double kMaxSpeedMetersPerSecond = 0.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 2;
    public static final double kPYController = 2;
    public static final double kPThetaController = 2;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond,
            kMaxAngularSpeedRadiansPerSecondSquared);

    // default end velocity is zero.
    public static final TrajectoryConfig kTrajectoryConfig = new TrajectoryConfig(
            kMaxSpeedMetersPerSecond,
            kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(Drivetrain.kDriveKinematics);

    // turn 90 degrees in place
    public static final Trajectory kTurnLeft = new Trajectory(
            List.of(
                    new MyTrajectoryState(0, 0, 0, new Pose2d(0, 0, new Rotation2d(1, 0)), 0),
                    new MyTrajectoryState(2, 0, 0, new Pose2d(0, 0, new Rotation2d(0, 1)), 0)));

    // do a square, pointing to the inside
    public static final Trajectory kSquareTrajectory = new Trajectory(
            List.of(
                    new MyTrajectoryState(0, 0, 0, new Pose2d(0, 0, new Rotation2d(0, 0)), 0),
                    new MyTrajectoryState(2, 0.25, 0, new Pose2d(0, 0, new Rotation2d(1, 1)), 0),
                    new MyTrajectoryState(4, 0.25, 0, new Pose2d(0.5, 0, new Rotation2d(0, 1)), 0),
                    new MyTrajectoryState(6, 0.25, 0, new Pose2d(1, 0, new Rotation2d(-1, 1)), 0),
                    new MyTrajectoryState(8, 0.25, 0, new Pose2d(1, 0.5, new Rotation2d(-1, 0)), 0),
                    new MyTrajectoryState(10, 0.25, 0, new Pose2d(1, 1, new Rotation2d(-1, -1)), 0),
                    new MyTrajectoryState(12, 0.25, 0, new Pose2d(0.5, 1, new Rotation2d(0, -1)),
                            0),
                    new MyTrajectoryState(14, 0.25, 0, new Pose2d(0, 1, new Rotation2d(1, -1)), 0),
                    new MyTrajectoryState(16, 0.25, 0, new Pose2d(0, 0.5, new Rotation2d(1, 0)), 0),
                    new MyTrajectoryState(18, 0.25, 0, new Pose2d(0, 0, new Rotation2d(1, 1)), 0),
                    new MyTrajectoryState(20, 0, 0, new Pose2d(0, 0, new Rotation2d(0, 0)), 0)));

    // simple strafing
    public static final Trajectory kExampleTrajectory = new Trajectory(
            List.of(
                    new MyTrajectoryState(0, 0, 0, new Pose2d(0, 0, new Rotation2d(0, 0)), 0),
                    new MyTrajectoryState(1, 0, 0, new Pose2d(0, 0, new Rotation2d(1, 2)), 0),
                    new MyTrajectoryState(3, 0.4, 0, new Pose2d(0.8, 0, new Rotation2d(0, 2)), 0),
                    new MyTrajectoryState(5, 0, 0, new Pose2d(1.6, 0, new Rotation2d(-1, 2)), 0),
                    new MyTrajectoryState(7, 0.4, 0, new Pose2d(0.8, 0, new Rotation2d(0, 2)), 0),
                    new MyTrajectoryState(9, 0, 0, new Pose2d(0, 0, new Rotation2d(1, 2)), 0),
                    new MyTrajectoryState(11, 0.4, 0, new Pose2d(0.8, 0, new Rotation2d(0, 2)), 0),
                    new MyTrajectoryState(13, 0, 0, new Pose2d(1.6, 0, new Rotation2d(-1, 2)), 0),
                    new MyTrajectoryState(15, 0.4, 0, new Pose2d(0.8, 0, new Rotation2d(0, 2)), 0),
                    new MyTrajectoryState(17, 0, 0, new Pose2d(0, 0, new Rotation2d(1, 2)), 0),
                    new MyTrajectoryState(19, 0.4, 0, new Pose2d(0.8, 0, new Rotation2d(0, 2)), 0),
                    new MyTrajectoryState(21, 0, 0, new Pose2d(1.6, 0, new Rotation2d(-1, 2)), 0),
                    new MyTrajectoryState(23, 0.4, 0, new Pose2d(0.8, 0, new Rotation2d(0, 2)), 0),
                    new MyTrajectoryState(25, 0, 0, new Pose2d(0, 0, new Rotation2d(1, 2)), 0)
            //
            ));

    public static final PIDController kXController = new PIDController(kPXController, 0, 0);
    public static final PIDController kYController = new PIDController(kPYController, 0, 0);
    public static final ProfiledPIDController kThetaController = new ProfiledPIDController(
            kPThetaController, 0, 0, kThetaControllerConstraints);

    private static final double kMaxWheelSpeedMS = 0.4;
    // private static final double kMaxWheelSpeedMS = 0.1;

    private final Drivetrain m_drivetrain;
    private final Timer m_timer;
    private final Trajectory m_trajectory;
    private final HolonomicDriveController m_controller;
    private final SwerveDriveKinematics m_kinematics;
    private final TrajectoryConfig m_config;
    private final Translation2d m_aimingPoint;
    private State m_desiredState;
    private double m_curTime;

    public MySwerveCommand(Drivetrain drivetrain) {
        m_timer = new Timer();
        m_kinematics = Drivetrain.kDriveKinematics;
        m_aimingPoint = new Translation2d(5, 2);
        m_config = new TrajectoryConfig(1, 1).setKinematics(m_kinematics);
        m_config.addConstraint(new TrackingConstraint(m_kinematics, kMaxWheelSpeedMS, m_aimingPoint));

        // m_trajectory = kExampleTrajectory;
        // m_trajectory = kSquareTrajectory;
        m_trajectory = squareOfTranslations(m_config);
        m_controller = new HolonomicDriveController(kXController, kYController, kThetaController);
        m_controller.setTolerance(new Pose2d(0.1, 0.1, new Rotation2d(0.1)));
        m_drivetrain = drivetrain;

        kThetaController.enableContinuousInput(-Math.PI, Math.PI);
        m_drivetrain.resetOdometry(m_trajectory.getInitialPose());
        m_desiredState = new Trajectory.State();
        m_curTime = 0;
        SmartDashboard.putData("My Swerve Command", this);
    }

    /**
     * 3x3 meter square centered at (0.5, 0.5), using translations, with start and
     * end poses to make a circle.
     */
    public static Trajectory squareOfTranslations(TrajectoryConfig config) {
        return TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(1, -1)), // pose is in the direction of the first waypoint
                List.of(
                        new Translation2d(1, 0),
                        new Translation2d(1, 1),
                        new Translation2d(0, 1)),
                new Pose2d(0, 0, new Rotation2d(1, -1)), // pose is in the direction from the last waypoint
                config);
    }

    @Override
    public void execute() {
        m_curTime = m_timer.get();
        m_desiredState = m_trajectory.sample(m_curTime);

        ChassisSpeeds targetChassisSpeeds = m_controller.calculate(m_drivetrain.getPose(), m_desiredState,
                m_desiredState.poseMeters.getRotation());
        var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

        m_drivetrain.setModuleStates(targetModuleStates);
    }

    @Override
    public void initialize() {
        // System.out.println("my serve command initialize");
        m_drivetrain.initialize();
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void end(boolean interrupted) {
        // System.out.println("my serve command end");
        // if (interrupted) {
        // System.out.println("my serve command interrupted");
        // }
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("x setpoint", () -> kXController.getSetpoint(), null);
        builder.addDoubleProperty("y setpoint", () -> kYController.getSetpoint(), null);
        builder.addDoubleProperty("theta setpoint", () -> kThetaController.getSetpoint().position, null);
        builder.addDoubleProperty("theta goal", () -> kThetaController.getGoal().position, null);
        builder.addBooleanProperty("is finished", () -> isFinished(), null);
        builder.addBooleanProperty("is scheduled", () -> isScheduled(), null);
        builder.addDoubleProperty("total runtime", () -> m_trajectory.getTotalTimeSeconds(), null);
        builder.addDoubleProperty("current time", () -> m_curTime, null);
        builder.addDoubleProperty("desired state rotation",
                () -> m_desiredState.poseMeters.getRotation().getRadians(),
                null);
        builder.addDoubleProperty("theta error", () -> kThetaController.getPositionError(), null);
        builder.addBooleanProperty("holo at ref", () -> m_controller.atReference(), null);
        builder.addDoubleProperty("x error", () -> kXController.getPositionError(), null);
        builder.addDoubleProperty("y error", () -> kYController.getPositionError(), null);
    }
}
