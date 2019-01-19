package frc.tests;

import frc.robot.utils.MathUtils;

class MathUtilsTest {

    @org.junit.jupiter.api.Test
    void calculateNeededGyroChange() {

    }

    @org.junit.jupiter.api.Test
    void checkTolerance() {
        double error = 15.0;
        final double tolerance = 10.0;

        System.out.println("Error: " + error + " " + MathUtils.checkTolerance(error, tolerance));
        error = 5.0;
        System.out.println("Error: " + error + " " + MathUtils.checkTolerance(error, tolerance));
        error = -5.0;
        System.out.println("Error: " + error + " " + MathUtils.checkTolerance(error, tolerance));
        error = 10.0;
        System.out.println("Error: " + error + " " + MathUtils.checkTolerance(error, tolerance));
        error = -15.0;
        System.out.println("Error: " + error + " " + MathUtils.checkTolerance(error, tolerance));
    }

    @org.junit.jupiter.api.Test
    void pixelsToInches(){
        System.out.println(MathUtils.pixelsToInches(106.0));
    }
}