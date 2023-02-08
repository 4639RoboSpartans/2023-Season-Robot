package frc.robot.util.math;

public class math {
    private math() {}

    public static double magnitude(double x, double y){
        return Math.sqrt(x * x + y * y);
    }

    public static double magnitude(vec2 v){
        return magnitude(v.x(), v.y());
    }

    /**
     * The cosine function in degrees mode
     */
    public static double cos(double degrees){
        return Math.cos(Math.toRadians(degrees));
    }
    
    public static double sin(double degrees){
        return Math.sin(Math.toRadians(degrees));
    }

    public static double tan(double degrees){
        return sin(degrees) / cos(degrees);
    }

    public static double atan2(double y, double x){
        return Math.toDegrees(Math.atan2(y, x));
    }

    public static double atan2(vec2 v){
        return atan2(v.y(), v.x());
    }

    public static vec2 rotateCCW(vec2 v, double degrees){
        return new vec2(
            v.x() * cos(degrees) - v.y() * sin(degrees), 
            v.x() * sin(degrees) + v.y() * cos(degrees)
        );
    }

    public static vec2 rotateCW(vec2 v, double degrees){
        return rotateCW(v, -degrees);
    }

    public static vec2 normalize(vec2 v){
        return v.times(1. / magnitude(v));
    }
}
