import static org.junit.Assert.assertEquals;
import org.junit.Test;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class FeedForwardTest {
    public static final double DELTA = 1e-2;

    @Test
    public void testToy() {
        assertEquals(1, 1);
    }

    @Test
    public void testSimpleMotorFeedforward() {
        SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0.1, 0.1);
        assertEquals(0.01, ff.calculate(0.1, 0.1, 0.02), DELTA);
        assertEquals(0.52, ff.calculate(0.1, 0.2, 0.02), DELTA);
        assertEquals(1.02, ff.calculate(0.1, 0.3, 0.02), DELTA);
        assertEquals(0.05, ff.calculate(0.5, 0.5, 0.02), DELTA);
        assertEquals(1.08, ff.calculate(0.7, 0.9, 0.02), DELTA);
        assertEquals(0.59, ff.calculate(0.8, 0.9, 0.02), DELTA);
        assertEquals(0.09, ff.calculate(0.9, 0.9, 0.02), DELTA);
    }
}
