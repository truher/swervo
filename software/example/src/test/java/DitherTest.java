import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;
import frc.math.Dither;

public class DitherTest {
    public static final double DELTA = 1e-2;

    @Test
    public void testToy() {
        assertEquals(1, 1);
    }

    @Test
    public void testDither() {
        Dither d = new Dither(0, -0.1, 0.1);  // seed for repeatability
        assertEquals(1, d.calculate(1), DELTA);
        assertEquals(-1, d.calculate(-1), DELTA);
        assertEquals(0.1, d.calculate(0.09), DELTA);
        assertEquals(-0.1, d.calculate(-0.09), DELTA);
        assertEquals(0.1, d.calculate(0), DELTA);
    }

    @Test
    public void testBias() {
        Dither d = new Dither(0, -0.1, 0.1);  // seed for repeatability
        int hi = 0;
        int lo = 0;
        for (int i = 0; i < 10000000; ++i) {
            if (d.calculate(0) > 0) {
                hi += 1;
            }
            else {
                lo += 1;
            }
        }
        int diff = hi - lo;
        assertTrue(String.format("hi: %d, lo: %d", hi, lo), diff < 100);
    }
}



