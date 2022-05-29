import static org.junit.Assert.assertEquals;
import org.junit.Test;

import edu.wpi.first.math.filter.LinearFilter;

// tests to find a filter that works the way i want.
// "Backwards Finite Difference" doesn't seem like it.
public class VelocityTest {
    public static final double DELTA = 1e-2;

    @Test
    public void testToy() {
        assertEquals(1, 1);
    }

    @Test
    public void testConstant() {
        LinearFilter x = LinearFilter.backwardFiniteDifference(1, 2, 1);
        assertEquals(0, x.calculate(0), DELTA);
        assertEquals(0, x.calculate(0), DELTA);
        assertEquals(0, x.calculate(0), DELTA);
        assertEquals(0, x.calculate(0), DELTA);
    }

    @Test
    public void testLinear() {
        LinearFilter x = LinearFilter.backwardFiniteDifference(1, 2, 1);
        assertEquals(0, x.calculate(0), DELTA);
        assertEquals(1, x.calculate(1), DELTA);
        assertEquals(1, x.calculate(2), DELTA);
        assertEquals(1, x.calculate(3), DELTA);
    }

    @Test
    public void testLinearNegative() {
        LinearFilter x = LinearFilter.backwardFiniteDifference(1, 2, 1);
        assertEquals(0, x.calculate(0), DELTA);
        assertEquals(-1, x.calculate(-1), DELTA);
        assertEquals(-1, x.calculate(-2), DELTA);
        assertEquals(-1, x.calculate(-3), DELTA);
    }

    @Test
    public void testLinearSteeper() {
        LinearFilter x = LinearFilter.backwardFiniteDifference(1, 2, 1);
        assertEquals(0, x.calculate(0), DELTA);
        assertEquals(2, x.calculate(2), DELTA);
        assertEquals(2, x.calculate(4), DELTA);
        assertEquals(2, x.calculate(6), DELTA);
    }

    @Test
    public void testStep() {
        LinearFilter x = LinearFilter.backwardFiniteDifference(1, 2, 1);
        assertEquals(0, x.calculate(0), DELTA);
        assertEquals(0, x.calculate(0), DELTA);
        assertEquals(1, x.calculate(1), DELTA);
        assertEquals(0, x.calculate(1), DELTA);
        assertEquals(0, x.calculate(1), DELTA);
    }

    @Test
    public void testBiggerWindow() {
        // surprising
        LinearFilter x = LinearFilter.backwardFiniteDifference(1, 3, 1);
        assertEquals(0, x.calculate(0), DELTA);
        assertEquals(0, x.calculate(0), DELTA);
        assertEquals(1.5, x.calculate(1), DELTA); // ?
        assertEquals(-0.5, x.calculate(1), DELTA);
        assertEquals(0, x.calculate(1), DELTA);
        assertEquals(0, x.calculate(1), DELTA);
        assertEquals(0, x.calculate(1), DELTA);
        assertEquals(0, x.calculate(1), DELTA);
    }

    @Test
    public void testMuchBiggerWindow() {
        LinearFilter x = LinearFilter.backwardFiniteDifference(1, 5, 1);
        assertEquals(0, x.calculate(0), DELTA);
        assertEquals(0, x.calculate(0), DELTA);
        assertEquals(2.08, x.calculate(1), DELTA);
        assertEquals(-1.92, x.calculate(1), DELTA);
        assertEquals(1.08, x.calculate(1), DELTA);
        assertEquals(-0.25, x.calculate(1), DELTA);
        assertEquals(0, x.calculate(1), DELTA);
        assertEquals(0, x.calculate(1), DELTA);
        assertEquals(0, x.calculate(1), DELTA);
        assertEquals(0, x.calculate(1), DELTA);
    }

    @Test
    public void testAverageDifferenceConstant() {
        LinearFilter x = new LinearFilter(new double[] { 0.5, -0.5 }, new double[0]);
        assertEquals(0, x.calculate(0), DELTA);
        assertEquals(0, x.calculate(0), DELTA);
        assertEquals(0, x.calculate(0), DELTA);
        assertEquals(0, x.calculate(0), DELTA);
    }

    @Test
    public void testAverageDifferenceLinear() {
        LinearFilter x = new LinearFilter(new double[] { 1, -1 }, new double[0]);
        assertEquals(0, x.calculate(0), DELTA);
        assertEquals(1, x.calculate(1), DELTA);
        assertEquals(1, x.calculate(2), DELTA);
        assertEquals(1, x.calculate(3), DELTA);
    }

    @Test
    public void testAverageDifferenceSteeper() {
        LinearFilter x = new LinearFilter(new double[] { 1, -1 }, new double[0]);
        assertEquals(0, x.calculate(0), DELTA);
        assertEquals(2, x.calculate(2), DELTA);
        assertEquals(2, x.calculate(4), DELTA);
        assertEquals(2, x.calculate(6), DELTA);
    }

    @Test
    public void testAverageDifferenceStep() {
        LinearFilter x = new LinearFilter(new double[] { 1, -1 }, new double[0]);
        assertEquals(0, x.calculate(0), DELTA);
        assertEquals(0, x.calculate(0), DELTA);
        assertEquals(1, x.calculate(1), DELTA);
        assertEquals(0, x.calculate(1), DELTA);
    }

    @Test
    public void testAverageDifferenceStepLonger() {
        LinearFilter x = new LinearFilter(new double[] { 0.5, 0, -0.5 }, new double[0]);
        assertEquals(0, x.calculate(0), DELTA);
        assertEquals(0, x.calculate(0), DELTA);
        assertEquals(0.5, x.calculate(1), DELTA);
        assertEquals(0.5, x.calculate(1), DELTA);
        assertEquals(0, x.calculate(1), DELTA);
        assertEquals(0, x.calculate(1), DELTA);
    }

    @Test
    public void testAverageDifferenceStepMuchLonger() {
        LinearFilter x = new LinearFilter(
            new double[] { 0.25, 0, 0, 0, -0.25 }, new double[0]);
        assertEquals(0, x.calculate(0), DELTA);
        assertEquals(0, x.calculate(0), DELTA);
        assertEquals(0.25, x.calculate(1), DELTA);
        assertEquals(0.25, x.calculate(1), DELTA);
        assertEquals(0.25, x.calculate(1), DELTA);
        assertEquals(0.25, x.calculate(1), DELTA);
        assertEquals(0, x.calculate(1), DELTA);
        assertEquals(0, x.calculate(1), DELTA);
        assertEquals(0, x.calculate(1), DELTA);
        assertEquals(0, x.calculate(1), DELTA);
    }

    @Test
    public void testAverageDifferenceLinearMuchLonger() {
        LinearFilter x = new LinearFilter(
            new double[] { 0.25, 0, 0, 0, -0.25 }, new double[0]);
        assertEquals(0, x.calculate(0), DELTA);
        assertEquals(0.25, x.calculate(1), DELTA);
        assertEquals(0.5, x.calculate(2), DELTA);
        assertEquals(0.75, x.calculate(3), DELTA);
        assertEquals(1, x.calculate(4), DELTA);
        assertEquals(1, x.calculate(5), DELTA);
        assertEquals(1, x.calculate(6), DELTA);
        assertEquals(1, x.calculate(7), DELTA);
        assertEquals(1, x.calculate(8), DELTA);
        assertEquals(1, x.calculate(9), DELTA);
    }

    @Test
    public void testAverage() {
        LinearFilter x = new LinearFilter(
            new double[] { 0.25, 0.25, 0.25, 0.25 }, new double[0]);
        assertEquals(0, x.calculate(0), DELTA);
        assertEquals(0.25, x.calculate(1), DELTA);
        assertEquals(0.75, x.calculate(2), DELTA);
        assertEquals(1.5, x.calculate(3), DELTA);
        assertEquals(2.25, x.calculate(3), DELTA);
        assertEquals(2.75, x.calculate(3), DELTA);
        assertEquals(3, x.calculate(3), DELTA);
        assertEquals(3, x.calculate(3), DELTA);
        assertEquals(3, x.calculate(3), DELTA);
        assertEquals(3, x.calculate(3), DELTA);
    }
}
