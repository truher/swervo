package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.sensors.FusedHeading;

public class Drivetrain extends SubsystemBase {
    private static final double kMaxSpeedMetersPerSecond = 0.54;
    // Base is an equilateral triangle 0.2794m (11 inches) on a side. Positive
    // directions are x forward, y left, theta counterclockwise, measured from the x
    // axis.
    private static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(0.1613, 0), // front
            new Translation2d(-0.0807, 0.1397), // left rear
            new Translation2d(-0.0807, -0.1397)); // right rear
    private final Module[] m_modules;
    private final SwerveDriveOdometry m_odometry;
    private final FusedHeading m_gyro;

    public Drivetrain() {
        m_modules = new Module[] {
                new Module(0, 0.48, 1),
                new Module(2, 0.43, 3),
                new Module(4, 0.85, 5)
        };
        m_odometry = new SwerveDriveOdometry(kDriveKinematics, new Rotation2d(0));
        m_gyro = new FusedHeading();
        m_gyro.reset();
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(
                m_gyro.get(),
                m_modules[0].getState(),
                m_modules[1].getState(),
                m_modules[2].getState());
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
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeedMetersPerSecond);
        m_modules[0].setDesiredState(swerveModuleStates[0]);
        m_modules[1].setDesiredState(swerveModuleStates[1]);
        m_modules[2].setDesiredState(swerveModuleStates[2]);
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
        m_modules[0].m_steer.initialize();
        m_modules[1].m_steer.initialize();
        m_modules[2].m_steer.initialize();
    }
}
