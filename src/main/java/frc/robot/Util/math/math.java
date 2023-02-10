package frc.robot.util.math;

public class math {
    private math() {}

    /**
     * @returns The magnitude of the vector &lt;x, y&gt;
     */
    public static double magnitude(double x, double y){
        return Math.sqrt(x * x + y * y);
    }

    /**
     * @returns The magnitude of the v
     */
    public static double magnitude(vec2 v){
        return magnitude(v.x(), v.y());
    }

    /**
     * The cosine function in degrees mode
     */
    public static double cos(double degrees){
        return Math.cos(Math.toRadians(degrees));
    }
    
    /**
     * The sin function in degrees mode
     */
    public static double sin(double degrees){
        return Math.sin(Math.toRadians(degrees));
    }

    /**
     * The tan function in degrees mode
     */
    public static double tan(double degrees){
        return sin(degrees) / cos(degrees);
    }

    
    /**
     * The atan2 function in degrees mode
     * @returns the angle in degrees that the vector &lt;x, y&gt; makes with the x-axis, measured counterclockwise from positive x
     */
    public static double atan2(double y, double x){
        return Math.toDegrees(Math.atan2(y, x));
    }

    /**
     * The atan2 function in degrees mode
     * @returns the angle in degrees that v makes with the x-axis, measured counterclockwise from positive x
     */
    public static double atan2(vec2 v){
        return atan2(v.y(), v.x());
    }

    /**
     * @return v rotated counterclockwise by angle, measured in degrees
     */
    public static vec2 rotateCCW(vec2 v, double angle){
        return new vec2(
            v.x() * cos(angle) - v.y() * sin(angle), 
            v.x() * sin(angle) + v.y() * cos(angle)
        );
    }

    /**
     * @return v rotated clockwise by angle, measured in degrees
     */
    public static vec2 rotateCW(vec2 v, double degrees){
        return rotateCW(v, -degrees);
    }

    /**
     * @return The unit vector in v's direction
     */
    public static vec2 normalize(vec2 v){
        return v.times(1. / magnitude(v));
    }

    /**
     * The modulo function.
     * @returns a value v such that v is in [0, m) and v = x - N * m where N is an integer
     */
    public static double mod(double x, double m){
        return (x % m + m) % m;
    }

    /**
     * Constrains x to the range [min, max). values outside of the range will wrap around.
     * For min = 0, this behaves the same as mod
     * @returns a value v such that v is in [min, max) and v = x - N * (max - min) where N is an integer
     */
    public static double mod(double x, double min, double max){
        return mod(x - min, max - min) + min;
    }

    /**
     * Rounds x to the nearest multiple of precision
     */
    public static double round(double x, double precision){
        return Math.round(x / precision) * precision;
    }

    public static double max(double... arr){
        double max = Double.NEGATIVE_INFINITY;
        for(double i : arr) if(i > max) max = i;
        return max;
    }
}
