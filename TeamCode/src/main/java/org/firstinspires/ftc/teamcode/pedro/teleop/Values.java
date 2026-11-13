package org.firstinspires.ftc.teamcode.pedro.teleop;

public class Values {
    /// 360 = 13,668.13;57.72, 56.13
    public static double goalX = 57.72, goalY = 56.13;
    public static Modes mode = Modes.INTAKING;
    public static Team team = Team.BLUE;
    public static boolean init = true,rumble=false;

    public static final double Turretpos = 0;
    public static final double HOOD_MAX = 0.37, HOOD_MIN = 0;

    public static final double LIMITER_OPEN = 0.7, LIMITER_CLOSED = 0.0;
    public enum Modes {
        INTAKING,
        SHOOTING,

    }

    public enum Team {
        RED,
        BLUE
    }

}
