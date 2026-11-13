package org.firstinspires.ftc.teamcode.pedro.teleop.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;

public class Turret {
    public static final double TICKS_PER_DEGREE = 6834.0 / 180.0; // ~37.9667 ticks/deg
    public static final double HALF_TURRET_RANGE_TICKS = 6834.0;  // 180 degrees in ticks
    public static final double FULL_TURRET_RANGE_TICKS = 13668.0;
    public static final double TICKS_PER_RADIAN = 6834.0 / Math.PI; // ~2175.33 ticks/rad

    public static double tP = 0.0001;
    public static double tI = 0.0;
    public static double tD = 0.0;
    public static double tF = 0.0;

    public static double ticksToDegrees(double ticks) {
        return ticks / TICKS_PER_DEGREE;
    }

    public static double degreesToTicks(double degrees) {
        return degrees * TICKS_PER_DEGREE;
    }

    private final PIDFController pid = new PIDFController(tP, tI, tD, tF);

    public double Autoaim(double robotX, double robotY, double heading, double goalx, double goaly) {
        double dx = goalx - robotX;
        double dy = goaly - robotY;
        double angle = Math.atan2(dy, dx);
        double reqAngle = angle - heading;
        reqAngle = Math.atan2(Math.sin(reqAngle), Math.cos(reqAngle));
        return reqAngle * TICKS_PER_RADIAN;
    }


    public double Turret_PID(Hardware hardware, double targetTicks) {
        double current = hardware.turret.getCurrentPosition();
        pid.setPIDF(tP, tI, tD, tF);

        double power = pid.calculate(current, targetTicks);
        power = Range.clip(power, -1.0, 1.0);
        double servoPos1 = (power + 1.0) / 2.0;
        double servoPos2 = 1.0 - servoPos1;

        hardware.turret1.setPosition(servoPos1);
        hardware.turret2.setPosition(servoPos2);


        return power;
    }
}
