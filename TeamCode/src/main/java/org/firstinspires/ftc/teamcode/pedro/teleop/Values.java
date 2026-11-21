package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.pedropathing.math.Pose;

public class Values {
    /// 360 = 13,668.13;57.72, 56.13
    public static boolean farGoal = false;
    public static double offsetAngle=0,txRaw=0,tx = 0,lastTxRaw=0, llOffset = 0;
    public static boolean txStale = false;
    public static double turretOverride=0;
    public static double flowerRampOff = 0.36; ///range is 0.8 0.9 and 1.0
    public static double flowerRampOn = 0.56;
    public static double angle = 0.85;
    public static Pose robotPose = null;
    public static double filteredX = 0;

    public static double goalX = 0, goalY = 0;
    public static final double FAR_ON = 74.0, FAR_OFF = 66.0;
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
