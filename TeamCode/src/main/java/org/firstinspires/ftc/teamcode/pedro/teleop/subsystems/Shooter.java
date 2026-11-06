package org.firstinspires.ftc.teamcode.pedro.teleop.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.controller.PIDFController;

public class Shooter {
    public static double flywheelOffset = 0;
    public static boolean lastDpadUp = false;
    public static boolean lastDpadDown = false;
    public static class flywheel_Values {
        public static double fP = 0.005;
        public static double fI = 0.000001;
        public static double fD = 0.0000001;
        public static double fF = 0.00045;
        public static double fV = 0;
        public static double fS = 0;
        public static PIDFController flywheelPIDController = new PIDFController(fP, fI, fD, fF);

        public static double kV = 0;
        public static double kS = 0;


        public static double flywheelTarget=1500;
    }

    public double flywheel(DcMotorEx m1, double target) {
        double v1 = m1.getVelocity();
        double curr = flywheel_Values.flywheelPIDController.calculate(v1, target);
        m1.setPower(curr);
        return v1;
    }
}

