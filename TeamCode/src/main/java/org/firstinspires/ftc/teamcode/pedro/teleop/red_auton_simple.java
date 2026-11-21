package org.firstinspires.ftc.teamcode.pedro.teleop;

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
    private final Pose start = poseFactory.of(68.3903, 4.5826, 180);
    private final Pose grab = poseFactory.of(8.7746, 6.2198, 180);
    private final Pose park = poseFactory.of(6.5113, 97.5835, -45.9476);
    private final Pose parkcontrol = poseFactory.of(36.4744, 66.6125, 0);

    public Path path1() { return Paths.line(start, grab).constant(grab); }
    public Path path2() { return Paths.curve(grab, parkcontrol, park).reverseTangent(); }

    public void setPathState(int newState) {
        pathState = newState;
        pathTimer.reset();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // STEP 1: SCORE PRELOAD
                spinning = true; // Spools up flywheel in loop()

                // Spool up for SPOOL_UP_MS, then open limiter & run transfer to shoot
                if (pathTimer.milliseconds() > SPOOL_UP_MS && pathTimer.milliseconds() <= SPOOL_UP_MS + PRELOAD_BURST_MS) {
                    shoot();
                } else if (pathTimer.milliseconds() > SPOOL_UP_MS + PRELOAD_BURST_MS) {
                    // Preload finished -> stop shooter & start driving to garden
                    stopShooting();
                    spinning = false;
                    follower.follow(path1());
                    setPathState(1);
                } else {
                    stopShooting();
                }
                break;

            case 1: // STEP 2: AT GARDEN -> Intake briefly, then drive to park
                if (!follower.isBusy()) {
                    intake();
                    if (pathTimer.milliseconds() > GARDEN_DWELL_MS) {
                        move();
                        follower.follow(path2());
                        setPathState(2);
                    }
                }
                break;

            case 2: // STEP 3: ARRIVED AT PARK -> Shut down mechanisms
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

    /** Opens limiter and runs intake + transfer to feed pollen into flywheel. */
    public void shoot() {
        hardware.limiter.setPosition(Values.LIMITER_OPEN);
        hardware.flower.setPosition(Values.flowerRampOff);
        hardware.intake.setPower(1.0);
        hardware.transfer.setPower(1.0);
    }

    /** Closes limiter and stops intake + transfer motors. */
    public void stopShooting() {
        hardware.limiter.setPosition(Values.LIMITER_CLOSED);
        hardware.intake.setPower(0);
        hardware.transfer.setPower(0);
    }

    /** Intakes pollen from garden. */
    public void intake() {
        hardware.limiter.setPosition(Values.LIMITER_CLOSED);
        hardware.intake.setPower(1.0);
        hardware.transfer.setPower(1.0);
        hardware.flower.setPosition(Values.flowerRampOff);
    }

    /** Holds mechanisms in neutral/closed position while driving. */
    public void move() {
        hardware.limiter.setPosition(Values.LIMITER_CLOSED);
        hardware.intake.setPower(0);
        hardware.transfer.setPower(0);
        hardware.flower.setPosition(Values.flowerRampOff);
    }

    @Override
    public void init() {
        pathTimer = new ElapsedTime();
        follower = Constants.create(hardwareMap);
        hardware = new Hardware(hardwareMap);
        shooter = new Shooter();
        turret = new Turret();

        // Zero Turret Encoder
        hardware.turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hardware.turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        Values.team = Values.Team.RED;
        Values.farGoal = false;
        Values.goalX = 57.56;
        Values.goalY = 58.0;
        Shooter.flywheelOffset = 0;
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
    public void init_loop() {
        turret.Turret_PID(hardware, 0);
    }

    @Override
    public void start() {
        turret.filteredX = 0;
        Values.turretOverride = 0;
        hardware.ll.pipelineSwitch(1); // RED Limelight pipeline
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

        // Flywheel Spool Up & Control
        if (spinning) {
            flywheelTarget = Shooter.rpmComp(dist) + Shooter.flywheelOffset;
            currentVel = shooter.flywheel(hardware.flywheel, flywheelTarget);
        } else {
            flywheelTarget = 0;
            hardware.flywheel.setPower(0);
            currentVel = Math.abs(hardware.flywheel.getVelocity());
        }

        hardware.hood.setPosition(Range.clip(Shooter.hoodpos, 0.01, 0.37));

        // Turret Auto-Aim & PID Loop
        lastTurretTarget = turret.Autoaim(botX, botY, botHeading, Values.goalX, Values.goalY, hardware.ll);
        turret.Turret_PID(hardware, lastTurretTarget);

        // Update Path State Machine
        autonomousPathUpdate();

        // Telemetry
        telemetry.addData("Path State", pathState);
        telemetry.addData("x", botX);
        telemetry.addData("y", botY);
        telemetry.addData("dist", dist);
        telemetry.addData("flywheel tgt/cur", flywheelTarget + " / " + currentVel);
        telemetry.addData("turret err", lastTurretTarget - hardware.turret.getCurrentPosition());
        if (follower.currentPath() != null) {
            telemetry.addData("Path distance remaining", follower.distanceToEndpoint());
        }
        telemetry.update();
    }
}

