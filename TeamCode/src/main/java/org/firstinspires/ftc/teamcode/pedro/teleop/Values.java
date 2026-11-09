package org.firstinspires.ftc.teamcode.pedro.teleop;

public class Values {
    public static Modes mode = Modes.INTAKING;
    public static Team team = Team.BLUE;
    public static boolean init = true,rumble=false;

    public static final double Turretpos = 0;

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
