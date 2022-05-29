package frc.math;

import java.util.SplittableRandom;

/**
 * Dither around a deadband.
 * 
 * I wrote this before I discovered how to turn on deadband correction in the
 * PWM controller, duh.  There's still a small frictional deadband, though,
 * so it might turn out to be useful.
 */
public class Dither {
    private final double m_min;
    private final double m_max;
    private final SplittableRandom m_random;

    /**
     * Dither around the deadband specified by min and max.
     */
    public Dither(double min, double max) {
        this(42, min, max);
    }

    /**
     * Dither around the deadband specified by min and max. Random seed for testing.
     */
    public Dither(long seed, double min, double max) {
        m_min = min;
        m_max = max;
        m_random = new SplittableRandom(seed);
    }

    /**
     * If the input is outside the deadband, return it.
     * Otherwise return one of the bounds, randomly selected
     * based on the input position in the deadband.
     */
    public double calculate(double in) {
        if (in <= m_min)
            return in;
        if (in >= m_max)
            return in;
        double r = m_random.nextDouble(m_min, m_max);
        if (in >= r)
            return m_max;
        return m_min;
    }
}
