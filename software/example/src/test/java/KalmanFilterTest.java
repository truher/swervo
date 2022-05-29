import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.Test;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;

// for experiments with kalman filter sensor fusion
// gyro is fast and precise but integration yields drift
// magnetometer is accurate but imprecise
public class KalmanFilterTest {
    public static final double DELTA = 1e-2;

    @Test
    public void testToy() {
        assertEquals(1, 1);
    }

    @Test
    public void testOne() {
        // say this is a simple 1d system
        // state: position and velocity, so N2
        // input: force, so N1
        // output: say we just observe position, so N1
        double dtSec = 0.02;
        // A is the system matrix: how the
        // states evolve if you leave the system alone.
        // xdot = A * x.
        Matrix<N2, N2> A = Matrix.mat(Nat.N2(), Nat.N2()).fill(0, 1, 0, 0);
        // B is the input matrix:
        // xdot = B * u for some input u, say it's force,
        // so it doesn't affect position but it does
        // affect velocity
        double m = 1;
        Matrix<N2, N1> B = Matrix.mat(Nat.N2(), Nat.N1()).fill(0, 1 / m);
        // C is the output matrix:
        // y = C * x so just select the x position
        Matrix<N1, N2> C = Matrix.mat(Nat.N1(), Nat.N2()).fill(1, 0);
        // D is the "feedthrough" of inputs directly to
        // outputs, and it should be zero.
        Matrix<N1, N1> D = Matrix.mat(Nat.N1(), Nat.N1()).fill(0);
        LinearSystem<N2, N1, N1> ls = new LinearSystem(A, B, C, D);

        // stdevs we expect from the model
        Matrix<N2, N1> stateStdDevs = Matrix.mat(Nat.N2(), Nat.N1()).fill(0.01, 0.01);
        // stdevs we expect from observations; the gaussian below is 1
        Matrix<N1, N1> outputStdDevs = Matrix.mat(Nat.N1(), Nat.N1()).fill(1);

        KalmanFilter<N2, N1, N1> kf = new KalmanFilter(Nat.N2(), Nat.N1(), ls, stateStdDevs, outputStdDevs, dtSec);
        // set initial state = moving
        double velocity = 1;
        double position = 0;
        kf.setXhat(Matrix.mat(Nat.N2(), Nat.N1()).fill(position, velocity));
        // check what we just said
        assertEquals(0, kf.getXhat().get(0, 0), DELTA);
        assertEquals(1, kf.getXhat().get(1, 0), DELTA);

        Random r = new Random();
        // run some observations through it
        System.out.printf(
                "i position velocity positionObservation predictedPosition predictedVelocity correctedPosition correctedVelocity\n");
        for (int i = 0; i < 100; ++i) {
            position += velocity * dtSec;
            // no input
            Matrix<N1, N1> controlInput = Matrix.mat(Nat.N1(), Nat.N1()).fill(0);
            // given no input, predict position
            kf.predict(controlInput, dtSec);
            double predictedPosition = kf.getXhat().get(0, 0);
            double predictedVelocity = kf.getXhat().get(1, 0);
            // correct the model given the actual observation
            // position moves at constant speed
            double positionObservation = position + 0.1 * r.nextGaussian();
            Matrix<N1, N1> observedOutput = Matrix.mat(Nat.N1(), Nat.N1()).fill(positionObservation);
            kf.correct(controlInput, observedOutput);
            double correctedPosition = kf.getXhat().get(0, 0);
            double correctedVelocity = kf.getXhat().get(1, 0);
            System.out.printf("%d %f %f %f %f %f %f %f\n",
                    i, position, velocity, positionObservation,
                    predictedPosition, predictedVelocity,
                    correctedPosition, correctedVelocity);
        }
        System.out.flush();
    }

    @Test
    public void testTwo() {
        // two measurements this time, y is [position, velocity] since the
        // magnetometer measures position and the gyro measures velocity.
        double m = 1; // mass
        double dtSec = 0.02;
        Matrix<N2, N2> A = Matrix.mat(Nat.N2(), Nat.N2()).fill(0, 1, 0, 0);
        Matrix<N2, N1> B = Matrix.mat(Nat.N2(), Nat.N1()).fill(0, 1 / m);
        Matrix<N2, N2> C = Matrix.mat(Nat.N2(), Nat.N2()).fill(1, 0, 0, 1);
        Matrix<N2, N1> D = Matrix.mat(Nat.N2(), Nat.N1()).fill(0, 0);

        LinearSystem<N2, N1, N2> ls = new LinearSystem(A, B, C, D);

        Matrix<N2, N1> stateStdDevs = Matrix.mat(Nat.N2(), Nat.N1()).fill(0.01, 0.01);
        Matrix<N2, N1> outputStdDevs = Matrix.mat(Nat.N2(), Nat.N1()).fill(1, 1);

        KalmanFilter<N2, N1, N2> kf = new KalmanFilter(Nat.N2(), Nat.N2(), ls, stateStdDevs, outputStdDevs, dtSec);

        double velocity = 1;
        double position = 0;
        kf.setXhat(Matrix.mat(Nat.N2(), Nat.N1()).fill(position, velocity));

        assertEquals(0, kf.getXhat().get(0, 0), DELTA);
        assertEquals(1, kf.getXhat().get(1, 0), DELTA);

        Random r = new Random();
        // run some observations through it
        System.out.printf(
                "i position velocity positionObservation velocityObservation predictedPosition predictedVelocity correctedPosition correctedVelocity\n");
        for (int i = 0; i < 100; ++i) {
            position += velocity * dtSec;

            Matrix<N1, N1> controlInput = Matrix.mat(Nat.N1(), Nat.N1()).fill(0);

            kf.predict(controlInput, dtSec);
            double predictedPosition = kf.getXhat().get(0, 0);
            double predictedVelocity = kf.getXhat().get(1, 0);

            // say position is noisier than velocity
            double positionObservation = position + 0.5 * r.nextGaussian();
            double velocityObservation = velocity + 0.1 * r.nextGaussian();

            Matrix<N2, N1> observedOutput = Matrix.mat(Nat.N2(),
                    Nat.N1()).fill(positionObservation, velocityObservation);
            kf.correct(controlInput, observedOutput);
            double correctedPosition = kf.getXhat().get(0, 0);
            double correctedVelocity = kf.getXhat().get(1, 0);
            System.out.printf("%d %f %f %f %f %f %f %f %f\n",
                    i, position, velocity, positionObservation, velocityObservation,
                    predictedPosition, predictedVelocity,
                    correctedPosition, correctedVelocity);
        }
        System.out.flush();

    }
}
