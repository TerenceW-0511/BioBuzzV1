package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.teleop.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.pedro.teleop.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedro.teleop.subsystems.Turret;

@Autonomous(name = "Red Auton StateMachine", group = "Autonomous")
public class Red_auton extends OpMode {
    private Follower follower;
    public Hardware hardware;
    public Shooter shooter;
    public Turret turret;

    private ElapsedTime pathTimer;
    private int pathState = 0;

    // shoot gating
    public static double RPM_TOL = 30;          // ticks/s
    public static double TURRET_TOL = 25;       // ticks (~0.66 deg)
    public static double SETTLE_MS = 150;       // limiter open -> feed
    public static double BURST_MS = 1500;       // tuned feed time for 3 flower pollen
    public static double PRELOAD_BURST_MS = 2000; // 4 preload pollen -> longer; tune so all 4 feed
    public static double SHOOT_TIMEOUT_MS = 4000; // never stall auto
    public static double RAMP_DWELL_MS = 250;   // let flower ramp deploy before driving
    public static boolean PRELOAD_TIPS_HIVE = false; // TODO: field-test (see notes)

    private boolean spinning = false;
    private boolean feedingStarted = false;
    private double burstMs = BURST_MS;
    private double flywheelTarget = 0, currentVel = 0, lastTurretTarget = 0, dist = 0;

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(68.3903, 4.5826, -179);
    private final Pose grab_garden = poseFactory.of(8.246, 5.3387, -179);
    private final Pose move_flower = poseFactory.of(22.9377, 46.2805, -179);
    private final Pose controlmove_flower = poseFactory.of(49.412, 46.4754, 0);
    private final Pose grab_flower = poseFactory.of(12.3288, 46.3718, 179.507);
    private final Pose move_shoot = poseFactory.of(48.4709, 33.159, 98.0595);
    private final Pose controlmove_shoot = poseFactory.of(46.4678, 47.3051, 0);
    private final Pose move_flower2 = poseFactory.of(47.0661, 116.3134, -61.8985);
    private final Pose controlmove_flower2 = poseFactory.of(73.3075, 67.1707, 0);
    private final Pose grab_flower2 = poseFactory.of(46.8656, 128.672, -89.0706);
    private final Pose move_shoot2 = poseFactory.of(47.1, 115.8136, -89);
    private final Pose end = poseFactory.of(9.4847, 116.1001, 179.5636);

    public void setPathState(int newState) {
        pathState = newState;
        pathTimer.reset();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // SCORE PRELOAD FIRST: spin up + aim + gated burst, THEN drive to garden
                spinning = true;
                burstMs = PRELOAD_BURST_MS;
                shootBurst();
                if (burstDone()) {
                    move();
                    spinning = false;
                    feedingStarted = false;
                    if (PRELOAD_TIPS_HIVE) { Values.farGoal = true; Values.goalY = 88.0; }
                    follower.follow(path1());
                    setPathState(1);
                }
                break;

            case 1: // at garden -> intake
                if (!follower.isBusy()) {
                    intake();
                    setPathState(2);
                }
                break;

            case 2: // dwell 800ms, then curve toward flower
                if (pathTimer.milliseconds() > 800) {
                    follower.follow(path2());
                    setPathState(33);
                }
                break;

            case 33:
                if (!follower.isBusy()) {
                    move();
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.follow(path3());
                    setPathState(4);
                }
                break;

            case 4: // at flower 1: ramp ON, spin up, dwell, drive back intaking
                if (!follower.isBusy()) {
                    flower_intake();
                    spinning = true;
                    if (pathTimer.milliseconds() > RAMP_DWELL_MS) {
                        follower.follow(path4());
                        setPathState(5);
                    }
                }
                break;

            case 5: // shoot burst 1, gated
                burstMs = BURST_MS;
                shootBurst();
                if (burstDone()) {
                    move();
                    spinning = false;
                    feedingStarted = false;
                    follower.follow(path5());
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.follow(path6());
                    setPathState(8);
                }
                break;

            case 8: // at flower 2: ramp ON, spin up, dwell, drive back intaking
                if (!follower.isBusy()) {
                    flower_intake();
                    spinning = true;
                    if (pathTimer.milliseconds() > RAMP_DWELL_MS) {
                        follower.follow(path7());
                        setPathState(9);
                    }
                }
                break;

            case 9: // shoot burst 2, gated
                burstMs = BURST_MS;
                shootBurst();
                if (burstDone()) {
                    move();
                    spinning = false;
                    feedingStarted = false;
                    follower.follow(path8());
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    move();
                    spinning = false;
                    setPathState(-1);
                }
                break;

            case -1:
                break;
        }
    }

    /** Feed only when flywheel at speed AND turret on target; timeout so auto never stalls. */
    private void shootBurst() {
        double clippedTarget = Range.clip(lastTurretTarget, Turret.MIN_TURRET_TICKS, Turret.MAX_TURRET_TICKS);
        boolean ready = Math.abs(currentVel - flywheelTarget) < RPM_TOL
                && Math.abs(clippedTarget - hardware.turret.getCurrentPosition()) < TURRET_TOL;
        if (ready) {
            hardware.limiter.setPosition(Values.LIMITER_OPEN);
            hardware.flower.setPosition(Values.flowerRampOff);
            if (!feedingStarted) {
                feedingStarted = true;
                pathTimer.reset();
            }
            boolean feed = pathTimer.milliseconds() > SETTLE_MS;
            hardware.intake.setPower(feed ? 1 : 0);
            hardware.transfer.setPower(feed ? 1 : 0);
        } else {
            hardware.limiter.setPosition(Values.LIMITER_CLOSED);
            hardware.intake.setPower(0);
            hardware.transfer.setPower(0);
        }
    }

    private boolean burstDone() {
        return (feedingStarted && pathTimer.milliseconds() > SETTLE_MS + burstMs)
                || (!feedingStarted && pathTimer.milliseconds() > SHOOT_TIMEOUT_MS);
    }

    @Override
    public void init() {
        pathTimer = new ElapsedTime();
        follower = Constants.create(hardwareMap);
        hardware = new Hardware(hardwareMap);   // BEFORE any hardware access
        shooter = new Shooter();
        turret = new Turret();

        Values.team = Values.Team.RED;
        Values.farGoal = false;
        Values.goalX = 57.56;
        Values.goalY = 58.0;
        Shooter.flywheelOffset = 0;             // clear teleop trims for a deterministic auto
        Shooter.hoodpos = 0.37;

        hardware.sintake.setPosition(Values.angle);
        hardware.flower.setPosition(Values.flowerRampOff);
        hardware.limiter.setPosition(Values.LIMITER_CLOSED);
        hardware.hood.setPosition(0.37);
        hardware.intake.setPower(0);
        hardware.transfer.setPower(0);
        hardware.flywheel.setPower(0);

        hardware.ll.setPollRateHz(100);

        follower.setPose(start);
        follower.update();
    }

    @Override
    public void start() {
        hardware.ll.pipelineSwitch(1); // RED pipeline, same as teleop
        hardware.ll.start();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();

        double botX = follower.pose().x();
        double botY = follower.pose().y();
        double botHeading = follower.pose().heading();
        dist = Turret.Distance(botX, botY, Values.goalX, Values.goalY);

        if (spinning) {
            flywheelTarget = Shooter.rpmComp(dist) + Shooter.flywheelOffset;
            currentVel = shooter.flywheel(hardware.flywheel, flywheelTarget);
        } else {
            flywheelTarget = 0;
            hardware.flywheel.setPower(0);
            currentVel = Math.abs(hardware.flywheel.getVelocity());
        }

        hardware.hood.setPosition(Range.clip(Shooter.hoodpos, 0.01, 0.37));

        lastTurretTarget = turret.Autoaim(botX, botY, botHeading, Values.goalX, Values.goalY, hardware.ll);

        autonomousPathUpdate();

        turret.Turret_PID(hardware, lastTurretTarget);

        telemetry.addData("Path State", pathState);
        telemetry.addData("x", botX);
        telemetry.addData("y", botY);
        telemetry.addData("heading", Math.toDegrees(botHeading));
        telemetry.addData("dist", dist);
        telemetry.addData("flywheel tgt/cur", flywheelTarget + " / " + currentVel);
        telemetry.addData("turret err", lastTurretTarget - hardware.turret.getCurrentPosition());
        telemetry.addData("feeding", feedingStarted);
        if (follower.currentPath() != null) {
            telemetry.addData("Path distance remaining", follower.distanceToEndpoint());
        }
        telemetry.update();
    }

    public void intake() {
        hardware.limiter.setPosition(Values.LIMITER_CLOSED);
        hardware.intake.setPower(1);
        hardware.transfer.setPower(1);
        hardware.flower.setPosition(Values.flowerRampOff);
    }

    public void outtake() {
        hardware.limiter.setPosition(Values.LIMITER_OPEN);
        hardware.intake.setPower(1);
        hardware.transfer.setPower(1);
        hardware.flower.setPosition(Values.flowerRampOff);
    }

    public void flower_intake() {
        hardware.limiter.setPosition(Values.LIMITER_CLOSED);
        hardware.intake.setPower(1);
        hardware.transfer.setPower(1);
        hardware.flower.setPosition(Values.flowerRampOn);
    }

    public void move() {
        hardware.limiter.setPosition(Values.LIMITER_CLOSED);
        hardware.intake.setPower(0);
        hardware.transfer.setPower(0);
        hardware.flower.setPosition(Values.flowerRampOff);
    }

    public Path path1() { return Paths.line(start, grab_garden).tangent(); }
    public Path path2() { return Paths.curve(grab_garden, controlmove_flower, move_flower).constant(move_flower); }
    public Path path3() { return Paths.line(move_flower, grab_flower).tangent(); }
    public Path path4() { return Paths.curve(grab_flower, controlmove_shoot, move_shoot).reverseTangent(); }
    public Path path5() { return Paths.curve(move_shoot, controlmove_flower2, move_flower2).reverseTangent(); }
    public Path path6() { return Paths.line(move_flower2, grab_flower2).reverseTangent(); }
    public Path path7() { return Paths.line(grab_flower2, move_shoot2).constant(move_shoot2); }
    public Path path8() { return Paths.line(move_shoot2, end).tangent(); }
}
