package org.firstinspires.ftc.teamcode.pedro.teleop.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.controller.PIDFController;

import java.util.Map;
import java.util.TreeMap;

public class Shooter {
    public static double flywheelOffset = 0;
    public static boolean lastDpadUp = false;
    public static boolean lastDpadDown = false;
    public static boolean lastDpadleft = false;
    public static boolean lastDpadright = false;

    public static double hoodpos = 0.37;
    public static final double HOOD_MIN = 0.37;
    public static final double HOOD_MAX = 0;
    public static double HOOD_OFFSET = 0.01;;

    public static double SHOOTER_OFFSET = 25;

    public static class flywheel_Values {
        public static double fP = 0.0044;
        public static double fI = 0.00001;
        public static double fD = 0.000026;
        public static double fF = 0;
        public static double fV = 0.000335;
        public static double fS = 0.066;
        public static PIDFController flywheelPIDController = new PIDFController(fP, fI, fD, fF);

        public static double kV = 0;
        public static double kS = 0;


        public static double flywheelTarget=1500;
    }

    public double flywheel(DcMotorEx m1, double target) {
        double currentVel = Math.abs(m1.getVelocity());
        flywheel_Values.flywheelPIDController.setPIDF(
                flywheel_Values.fP,
                flywheel_Values.fI,
                flywheel_Values.fD,
                flywheel_Values.fF
        );

        double curr = (flywheel_Values.fV * target) + flywheel_Values.fS+flywheel_Values.flywheelPIDController.calculate(currentVel, target);
        m1.setPower(curr);
        return currentVel;
    }
/// 69.4345
//    public static final TreeMap<Double,Double> rpmLUT = new TreeMap<>(
//            Map.ofEntries(
//                    Map.entry(74.5051, 1500.0),
//                    Map.entry(69.4345,1500.0),
//                    Map.entry(51.57541835973636,1580.0),
//                    Map.entry(61.20058168254951,1690.0),
//                    Map.entry(72.20668678918508, 1890.0),
//                    Map.entry(85.50943734702756, 2150.0)
//            )
//    );
}

