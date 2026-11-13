package org.firstinspires.ftc.teamcode.pedro.teleop.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.controller.PIDFController;

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
}

