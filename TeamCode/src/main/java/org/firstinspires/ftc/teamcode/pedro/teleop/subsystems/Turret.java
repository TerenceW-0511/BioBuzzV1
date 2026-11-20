package org.firstinspires.ftc.teamcode.pedro.teleop.subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PDController;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;

public class Turret {
    public static final double TICKS_PER_DEGREE = 6834.0 / 180.0; // ~37.9667 ticks/deg
    public static final double HALF_TURRET_RANGE_TICKS = 6834.0;  // 180 degrees in ticks
    public static final double FULL_TURRET_RANGE_TICKS = 13668.0;
    public static final double TICKS_PER_RADIAN = 6834.0 / Math.PI; // ~2175.33 ticks/rad

    public static final double MAX_TURRET_TICKS = 6000;
    public static final double MIN_TURRET_TICKS = -5500;

    public static double Tolerance = 50;

    public static double tP = 0.000375;

    public static double tI = 0.0;

//0.0000025 0.0000071
    public static double tD = 0.0000025;

    public static double ticksToDegrees(double ticks) {
        return ticks / TICKS_PER_DEGREE;
    }

    public static double degreesToTicks(double degrees) {
        return degrees * TICKS_PER_DEGREE;
    }

    private final PIDController pid = new PIDController(tP, tI, tD);
////

    public double Autoaim(double robotX, double robotY, double heading, double goalx, double goaly) {
        double dx = goalx - robotX;
        double dy = goaly - robotY;
        double angleToTarget = Math.atan2(dy, dx);
        double relativeAngle = angleToTarget - heading;
//        LLResult result= ll.getLatestResult();
//        Values.txRaw = result.getTx();
//        Values.tx = result.getTx();
        relativeAngle = Math.atan2(Math.sin(relativeAngle), Math.cos(relativeAngle));
        return relativeAngle * TICKS_PER_RADIAN;
    }

    //public double Autoaim(double robotX, double robotY, double heading, double goalx, double goaly, Limelight3A ll) {
//    double dx = goalx - robotX;
//    double dy = goaly - robotY;
//    double angleToTarget = Math.atan2(dy, dx);
//    double relativeAngle = angleToTarget - heading;
//    relativeAngle = Math.atan2(Math.sin(relativeAngle), Math.cos(relativeAngle));
//    double baseTargetTicks = relativeAngle * TICKS_PER_RADIAN;
//    double targetTicks;
//
//    if (ll != null) {
//        LLResult result = ll.getLatestResult();
//        Values.txRaw = (result != null) ? result.getTx() : 0;
//
//        double offsetAmt = 0;
//        if (Values.farCoded) {
//            offsetAmt = (Values.team == Values.Team.BLUE) ? 3 : -3;
//        }
//
//        if (!Values.txStale && result != null && result.isValid()) {
//            offsetAmt *= (Values.team == Values.Team.BLUE) ? 1 : -1;
//            Values.tx = result.getTx() - offsetAmt;
//
//            filteredX += test * Values.tx;
//            targetTicks = baseTargetTicks + filteredX;
//        } else {
//            Values.tx = 0;
//            targetTicks = baseTargetTicks;
//        }
//        Values.lastTxRaw = Values.txRaw;
//    } else {
//        targetTicks = baseTargetTicks;
//    }
//
//    targetTicks += Values.turretOverride;
//
//    return Range.clip(targetTicks, MIN_TURRET_TICKS, MAX_TURRET_TICKS);
//}

//    public static Pose fromPose3d(Pose3D original) {
//        Pose pose = new Pose(
//                toInches(original.getPosition().y) + 72,
//                -toInches(original.getPosition().x) + 72,
//                original.getOrientation().getYaw(AngleUnit.RADIANS) + Math.PI / 2
//        );
//        return pose.setHeading((pose.getHeading() + Math.PI) % (2 * Math.PI));
//    }

    public static double Distance(double botX, double botY, double goalX, double goalY) {
        double dx = botX - goalX;
        double dy = botY - goalY;
        double distance = Math.hypot(dx, dy);
        return distance;
    }

    public double Turret_PID(Hardware hardware, double targetTicks) {
        double current = hardware.turret.getCurrentPosition();
        targetTicks = Range.clip(targetTicks, MIN_TURRET_TICKS, MAX_TURRET_TICKS);
        pid.setPID(tP, tI, tD);

        double power = pid.calculate(current, targetTicks);

        if (Math.abs(targetTicks - current) < Tolerance) {
            power = 0;
        } else {
            power = Range.clip(power, -1.0, 1.0);
        }

        double servoPos1 = (power + 1.0) / 2.0;
        double servoPos2 = servoPos1;
        hardware.turret1.setPosition(servoPos1);
        hardware.turret2.setPosition(servoPos2);
        //skibidi
        return power;
    }
//    public void manualRelocalize(Follower follower){
//        double y = follower.getPose().getY();
//        if (Values.team==Values.Team.BLUE) {
//            if (y>50){
//                follower.setPose(new Pose(16.2,77.5,Math.toRadians(180)));
//            }else {
//                follower.setPose(new Pose(135 - 23.5, 6.5, Math.toRadians(0)));
//            }
//        }else{
//            if (y>50){
//                follower.setPose(new Pose(128,77.5,Math.toRadians(0)));
//            }else {
//                follower.setPose(new Pose(9+23.5,6.5,Math.toRadians(180)));
//            }
//
//        }
//        filteredX=0;
//        Values.turretOverride=0;
//        Values.tx=0;
//        Values.llOverride=0;
//    }
}
