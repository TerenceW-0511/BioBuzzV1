package org.firstinspires.ftc.teamcode.pedro.teleop.Autons;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.teleop.Values;
import org.firstinspires.ftc.teamcode.pedro.teleop.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.pedro.teleop.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedro.teleop.subsystems.Turret;

@Autonomous(name = "Red Auton Simple", group = "Autonomous")
public class red_auton_simple extends OpMode {
    private Follower follower;
    public Hardware hardware;
    public Shooter shooter;
    public Turret turret;

    private ElapsedTime pathTimer;
    private int pathState = 0;

    // Timing parameters (in milliseconds)
    public static double SPOOL_UP_MS = 400;       // Time for flywheel & turret to get ready
    public static double PRELOAD_BURST_MS = 2000; // Duration to feed all 4 preload pollen
    public static double GARDEN_DWELL_MS = 800;   // Duration to intake at garden

    private boolean spinning = false;
    private double flywheelTarget = 0, currentVel = 0, lastTurretTarget = 0, dist = 0;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(58.2908, 8, 90);
    private final Pose path1 = poseFactory.of(6.66, 96.2653, 90);
    private final Pose path1Control1 = poseFactory.of(6.4993, 60.2971, 0);

    public Path path1() {
        return Paths.curve(start, path1Control1, path1).constant(path1);
    }


    public void setPathState(int newState) {
        pathState = newState;
        pathTimer.reset();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                spinning = true;
                if (pathTimer.milliseconds()> 3400) {
                    hardware.limiter.setPosition(Values.LIMITER_OPEN);
                }
                if (pathTimer.milliseconds()> 4000) {
                    shoot();
                }

                if (pathTimer.milliseconds() > 5000) {
                    stopShooting();
                    spinning = false;
                    follower.follow(path1());
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    move();
                    spinning = false;
                    setPathState(-1);
                }
                break;

            case -1: // Auton Complete
                break;
        }
    }

    public void shoot() {
        hardware.intake.setPower(1.0);
        hardware.transfer.setPower(1.0);
    }

    public void stopShooting() {
        hardware.limiter.setPosition(Values.LIMITER_CLOSED);
        hardware.intake.setPower(0);
        hardware.transfer.setPower(0);
    }

    public void intake() {
        hardware.limiter.setPosition(Values.LIMITER_CLOSED);
        hardware.intake.setPower(1.0);
        hardware.transfer.setPower(1.0);
    }

    public void move() {
        hardware.limiter.setPosition(Values.LIMITER_CLOSED);
        hardware.intake.setPower(0);
        hardware.transfer.setPower(0);
    }

    @Override
    public void init() {
        pathTimer = new ElapsedTime();
        follower = Constants.create(hardwareMap);
        hardware = new Hardware(hardwareMap);
        shooter = new Shooter();
        turret = new Turret();

        hardware.turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hardware.turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        Values.team = Values.Team.RED;
        Values.farGoal = false;
        Values.goalX = 57.56;
        Values.goalY = 58.0;
        Shooter.flywheelOffset = 0;
        Shooter.hoodpos = 0.37;

        hardware.sintake.setPosition(1);
        hardware.limiter.setPosition(Values.LIMITER_CLOSED);
//        hardware.hood.setPosition(0.37);
        hardware.intake.setPower(0);
        hardware.transfer.setPower(0);

        hardware.ll.setPollRateHz(100);

        follower.setPose(start);
        follower.update();
    }

    @Override
    public void init_loop() {
        turret.Turret_PID(hardware, 0);
    }

    @Override
    public void start() {
        turret.filteredX = 0;
        Values.turretOverride = 0;
        hardware.ll.pipelineSwitch(1);
        hardware.ll.start();
        setPathState(0);
    }
    @Override
    public void stop() {
        if (follower != null) {
            Values.robotPose = follower.pose();
        }
    }

    @Override
    public void loop() {
        follower.update();

        double botX = follower.pose().x();
        double botY = follower.pose().y();
        double botHeading = follower.pose().heading();
        dist = Turret.Distance(botX, botY, Values.goalX, Values.goalY);
//        flywheelTarget = Shooter.rpmComp(dist) + Shooter.flywheelOffset;
//        currentVel = shooter.flywheel(hardware.flywheel, flywheelTarget);
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
        turret.Turret_PID(hardware, lastTurretTarget);

        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("x", botX);
        telemetry.addData("y", botY);
        telemetry.addData("heading", botHeading);
        telemetry.addData("dist", dist);
        telemetry.addData("flywheel tgt/cur", flywheelTarget + " / " + currentVel);
        telemetry.addData("turret err", lastTurretTarget - hardware.turret.getCurrentPosition());
        if (follower.currentPath() != null) {
            telemetry.addData("Path distance remaining", follower.distanceToEndpoint());
        }
        telemetry.update();
    }
}

