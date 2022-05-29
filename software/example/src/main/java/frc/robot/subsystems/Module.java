package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * A swerve module comprising a driving component and a turning component.
 */
public class Module {
    public final Turner m_steer;
    public final Driver m_drive;

    /**
     * @param steerChannel Same channel for PWM (motor) and DIO (duty cycle encoder)
     * @param steerOffset  Offset is measured in sensor units, [0,1]. To adjust the
     *                     module zero in the
     *                     positive (anticlockwise) direction, reduce the offset.
     * @param driveChannel Same channel for PWM (motor) and DIO (duty cycle encoder)
     */
    public Module(int steerChannel, double steerOffset, int driveChannel) {
        m_steer = new Turner(steerChannel, steerOffset);
        m_drive = new Driver(driveChannel);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_drive.getMeasurement(),
                new Rotation2d(m_steer.getPosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(m_steer.getPosition()));
        m_drive.setSetpoint(state.speedMetersPerSecond);
        m_steer.setGoal(state.angle.getRadians());
    }

}
