package frc.robot.subsystems;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.sensors.FusedHeading;

/** Triangular swerve base. */
public class Drivetrain extends SubsystemBase {
    private static final SwerveModuleState kQuiescentState = new SwerveModuleState();
    private static final double kMaxSpeedMetersPerSecond = 0.54;
    private static final double kMaxAllowedCrossTrackErrorMeters = 0.05;
    // Base is an equilateral triangle 0.2794m (11 inches) on a side. Positive
    // directions are x forward, y left, theta counterclockwise, measured from the x
    // axis.
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(0.1613, 0), // front
            new Translation2d(-0.0807, 0.1397), // left rear
            new Translation2d(-0.0807, -0.1397)); // right rear
    private final Module[] m_modules;
    private final FusedHeading m_gyro;
    // <num modules + 3, num modules + 3, num modules + 1>
    private final SwerveDrivePoseEstimator<N6, N6, N4> m_poseEstimator;

    public Drivetrain() {
        m_modules = new Module[] {
                new Module(0, 0.48, 1),
                new Module(2, 0.43, 3),
                new Module(4, 0.85, 5)
        };
        m_gyro = new FusedHeading();
        m_gyro.reset();
        Rotation2d gyroRotation = m_gyro.get();
        m_poseEstimator =
        new SwerveDrivePoseEstimator<N6, N6, N4>(
            Nat.N6(), 
            Nat.N6(),
            Nat.N4(),
                gyroRotation,
                new SwerveModulePosition[] {
                    m_modules[0].getPosition(),
                    m_modules[1].getPosition(),
                    m_modules[2].getPosition()
                },
                new Pose2d(0, 0, gyroRotation),
                kDriveKinematics,
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(5), 0.5, 0.5, 0.5),
                VecBuilder.fill(Units.degreesToRadians(0.01), 0.1, 0.1, 0.1),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
        SmartDashboard.putData("Drivetrain", this);
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_poseEstimator.update(
                m_gyro.get(),
                new SwerveModuleState[] {
                        m_modules[0].getState(),
                        m_modules[1].getState(),
                        m_modules[2].getState()
                },
                new SwerveModulePosition[] {
                        m_modules[0].getPosition(),
                        m_modules[1].getPosition(),
                        m_modules[2].getPosition()
                });
    }

    /**
     * Tries to keep the wheels pointing north, and moving forward slowly so I can
     * tell which way is forward.
     */
    public void keepWheelsPointingNorth() {
        Rotation2d yaw = m_gyro.get();
        SwerveModuleState desiredState = new SwerveModuleState(0.1, yaw.times(-1));
        m_modules[0].setDesiredState(desiredState);
        m_modules[1].setDesiredState(desiredState);
        m_modules[2].setDesiredState(desiredState);
    }

    /**
     * Actually do the swerving.
     * 
     * @param xSpeed from user input
     * @param ySpeed from user input
     * @param rot    from user input
     */
    public void drive(double xSpeed, double ySpeed, double rot) {
        SwerveModuleState[] swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.get()));

        setModuleStates(swerveModuleStates);

    }

    public void constrainCrossTrackError(SwerveModuleState[] swerveModuleStates) {
        double maxCTE = 0;
        for (int i = 0; i < 3; ++i) {
            SwerveModuleState s = swerveModuleStates[i];
            Module m = m_modules[i];
            double steerRadiansToGo = m.m_steer.getPosition() - s.angle.getRadians();
            double crossTrackErrorMeters = s.speedMetersPerSecond * Math.sin(steerRadiansToGo)
                    / (Turner.kMaxVelocity / Turner.kGearRatio);
            maxCTE = Math.max(maxCTE, crossTrackErrorMeters);
        }

        if (maxCTE > kMaxAllowedCrossTrackErrorMeters) {
            for (SwerveModuleState s : swerveModuleStates) {
                s.speedMetersPerSecond = s.speedMetersPerSecond * kMaxAllowedCrossTrackErrorMeters / maxCTE;
            }
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // System.out.println("set module states");

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kMaxSpeedMetersPerSecond);
        constrainCrossTrackError(desiredStates);
        // System.out.println(desiredStates[0].toString());
        // System.out.println(desiredStates[1].toString());
        // System.out.println(desiredStates[2].toString());

        m_modules[0].setDesiredState(desiredStates[0]);
        m_modules[1].setDesiredState(desiredStates[1]);
        m_modules[2].setDesiredState(desiredStates[2]);
    }

    public void zeroModules() {
        m_modules[0].setDesiredState(kQuiescentState);
        m_modules[1].setDesiredState(kQuiescentState);
        m_modules[2].setDesiredState(kQuiescentState);
    }

    // for drone mode, set angle goal directly
    public void setTurnGoal(double input) {
        for (Module module : m_modules) {
            module.m_steer.setGoal(input);
        }
    }

    // for normal mode, increment angle goal
    public void setTurnRate(double input) {
        for (Module module : m_modules) {
            module.m_steer.setTurnRate(input);
        }
    }

    public void setThrottle(double input) {
        for (Module module : m_modules) {
            module.m_drive.setThrottle(input);
        }
    }

    public void initialize() {
        m_modules[0].m_drive.initialize();
        m_modules[1].m_drive.initialize();
        m_modules[2].m_drive.initialize();
        m_modules[0].m_steer.initialize();
        m_modules[1].m_steer.initialize();
        m_modules[2].m_steer.initialize();
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("pose estimator rotation", () -> getPose().getRotation().getRadians(), null);
        builder.addDoubleProperty("gyro rotation", () -> m_gyro.get().getRadians(), null);
    }
}
