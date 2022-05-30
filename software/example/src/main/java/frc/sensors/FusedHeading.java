package frc.sensors;

import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.util.Unroller;

/**
 * Combine mag and gyro with Kalman filter.
 * 
 * Mag is NED, gyro is NWU, we produce NWU.
 */
public class FusedHeading implements Supplier<Rotation2d>, Sendable {
    private static final double kDtSec = 0.02;
    // State is [position, velocity, accel], so i can get a handle on the evolution of accel.
    private static final Matrix<N3, N3> kA = Matrix.mat(Nat.N3(), Nat.N3()).fill(0, 1, 0.5, 0, 0, 1, 0, 0, 0);
    // Input is torque (unused for now)
    private static final Matrix<N3, N1> kB = Matrix.mat(Nat.N3(), Nat.N1()).fill(0, 0, 1);
    // Output is [position, velocity] as measured by mag, gyro; accel is unmeasured.
    private static final Matrix<N2, N3> kC = Matrix.mat(Nat.N2(), Nat.N3()).fill(1, 0, 0, 0, 1, 0);
    // feedthrough is zero
    private static final Matrix<N2, N1> kD = Matrix.mat(Nat.N2(), Nat.N1()).fill(0, 0);
    // State position stdev
    // State velocity stdev
    // State accel stdev: set this low to try to make it smaller
    private static final Matrix<N3, N1> kStateStdDevs = Matrix.mat(Nat.N3(), Nat.N1()).fill(1, 3, 5);
    // Compass is pretty noisy, 0.1 radians
    // Gyro stdev is extremely low, 0.005 radians/sec
    private static final Matrix<N2, N1> kOutputStdDevs = Matrix.mat(Nat.N2(), Nat.N1()).fill(0.1, 0.1);
    // For now there is no Input.
    private static final Matrix<N1, N1> kControlInput = Matrix.mat(Nat.N1(), Nat.N1()).fill(0);

    private final Unroller m_mag;
    private final LSM6DSOX_I2C m_gyro;
    // 2 states, 1 input, 2 outputs
    private final LinearSystem<N3, N1, N2> m_system;
    private final KalmanFilter<N3, N1, N2> m_filter;

    public FusedHeading() {
        m_mag = new Unroller(new LIS3MDL_I2C());
        m_gyro = new LSM6DSOX_I2C();
        m_system = new LinearSystem<N3, N1, N2>(kA, kB, kC, kD);
        m_filter = new KalmanFilter<N3, N1, N2>(Nat.N3(), Nat.N2(), m_system, kStateStdDevs, kOutputStdDevs, kDtSec);
        System.out.println("====================== K ====================");
        System.out.println(m_filter.getK().toString());
        SmartDashboard.putData("heading", this);
    }

    public double getNEDMagRadians() {
        return m_mag.get().getRadians();
    }

    public double getNWUMagRadians() {
        return -1.0 * getNEDMagRadians();
    }

    public double getNWUGyroRadiansPerSec() {
        return m_gyro.getRate();
    }

    /**
     * Observations are [NWU radians, NWU radians/sec].
     */
    public Matrix<N2, N1> getObservations() {
        return Matrix.mat(Nat.N2(), Nat.N1()).fill(getNWUMagRadians(), getNWUGyroRadiansPerSec());
    }

    public void reset() {
        Matrix<N2, N1> obs = getObservations();
        m_filter.setXhat(0, obs.get(0,0));
        m_filter.setXhat(1, obs.get(1,0));
    }

    public Matrix<N3, N1> getState() {
        return m_filter.getXhat();
    }

    public double getPosition() {
        return getState().get(0, 0);
    }

    public double getVelocity() {
        return getState().get(1, 0);
    }

    public double getAcceleration() {
        return getState().get(2, 0);
    }

    /**
     * Yaw in radians referenced to magnetic north, NWU orientation.
     */
    @Override
    public Rotation2d get() {
        m_filter.predict(kControlInput, kDtSec);
        m_filter.correct(kControlInput, getObservations());
        return new Rotation2d(getPosition());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("model position", this::getPosition, null);
        builder.addDoubleProperty("model velocity", this::getVelocity, null);
        builder.addDoubleProperty("model acceleration", this::getAcceleration, null);
        builder.addDoubleProperty("unrolled mag nwu radians", this::getNWUMagRadians, null);
        builder.addDoubleProperty("gyro rate nwu radians per sec", this::getNWUGyroRadiansPerSec, null);

    }

}
