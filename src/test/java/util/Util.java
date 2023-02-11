package util;

import org.junit.jupiter.api.Assertions;

public class Util {
    public static double ACCEPTABLE_ERROR = 0.0001;
    public static void assertEquals(double a, double b, String message){
        Assertions.assertEquals(a, b, ACCEPTABLE_ERROR, message);
    }
}
