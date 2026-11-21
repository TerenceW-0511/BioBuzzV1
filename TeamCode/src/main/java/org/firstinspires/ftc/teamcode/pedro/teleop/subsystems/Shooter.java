package org.firstinspires.ftc.teamcode.pedro.teleop.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.pedro.teleop.Values;

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
    public static void manualRelocalize(Follower follower) {
        double y = follower.pose().y();

        double halfLength = 8.1765;
        double halfWidth  = 5.9465;

        if (Values.team == Values.Team.BLUE) {
            if (y > 50) {
                follower.setPose(new Pose(halfLength + 8.0, 77.5, Math.toRadians(180)));
            } else {
                follower.setPose(new Pose(144.0 - 23.5 - halfLength, halfWidth, Math.toRadians(0)));
            }
        } else {
            if (y > 50) {
                follower.setPose(new Pose(144.0 - (halfLength + 8.0), 77.5, Math.toRadians(0)));
            } else {
                follower.setPose(new Pose(23.5 + halfLength, halfWidth, Math.toRadians(180)));
            }
        }

        Values.filteredX = 0;
        Values.turretOverride = 0;
        Values.tx = 0;
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
    public static final TreeMap<Double,Double> rpmLUT = new TreeMap<>(
            Map.ofEntries(
                    Map.entry(83.9807,1655.0),
                    Map.entry(82.4326,1650.0),
                    Map.entry(75.4902,1600.0),
                    Map.entry(68.8208, 1575.0),
                    Map.entry(60.3188, 1525.0),
                    Map.entry(51.7581, 1510.0)
            )
    );
    public static double rpmComp(double dist) {

        if (dist <= rpmLUT.firstKey()) return rpmLUT.firstEntry().getValue();
        if (dist >= rpmLUT.lastKey())  return rpmLUT.lastEntry().getValue();

        Map.Entry<Double, Double> lower = rpmLUT.floorEntry(dist);
        Map.Entry<Double, Double> upper = rpmLUT.ceilingEntry(dist);

        if (lower == null || upper == null) {
            return rpmLUT.lastEntry().getValue();
        }

        if (lower.getKey().equals(upper.getKey())) {
            return lower.getValue();
        }

        double x0 = lower.getKey();
        double y0 = lower.getValue();
        double x1 = upper.getKey();
        double y1 = upper.getValue();

        double t = (dist - x0) / (x1 - x0);
        double newtarget = y0 + t * (y1 - y0);
        return newtarget;
    }
}

