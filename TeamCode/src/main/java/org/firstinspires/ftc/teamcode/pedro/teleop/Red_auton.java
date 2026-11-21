package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.teleop.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.pedro.teleop.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedro.teleop.subsystems.Turret;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Red Auton StateMachine", group = "Autonomous")
public class Red_auton extends OpMode {
    private Follower follower;
    public Hardware hardware;
    public Shooter shooter;
    public Turret turret;

    private Timer pathTimer;
    private int pathState = 0;

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
            case 0:
                follower.follow(path1());
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    hardware.sintake.setPosition(Values.angle);
                    hardware.flower.setPosition(Values.angle1);
                    hardware.intake.setPower(1.0);
                    hardware.transfer.setPower(1.0);
                    setPathState(2);
                }
                break;

            case 2:
                if (pathTimer.get(TimeUnit.MILLISECONDS) > 800) {
                    follower.follow(path2());
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.follow(path3());
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    hardware.intake.setPower(0);
                    hardware.transfer.setPower(0);
                    hardware.flywheel.setPower(0.9);

                    follower.follow(path4());
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    hardware.limiter.setPosition(Values.LIMITER_OPEN);
                    hardware.transfer.setPower(1.0);
                    setPathState(6);
                }
                break;

            case 6:
                if (pathTimer.get(TimeUnit.MILLISECONDS) > 1500) {
                    hardware.limiter.setPosition(Values.LIMITER_CLOSED);
                    hardware.transfer.setPower(0);
                    hardware.flywheel.setPower(0);

                    hardware.intake.setPower(1.0);

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

            case 8:
                if (!follower.isBusy()) {
                    hardware.intake.setPower(0);
                    hardware.flywheel.setPower(0.9);
                    follower.follow(path7());
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    hardware.limiter.setPosition(Values.LIMITER_OPEN);
                    hardware.transfer.setPower(1.0);
                    setPathState(10);
                }
                break;

            case 10:
                if (pathTimer.get(TimeUnit.MILLISECONDS) > 1500) {
                    hardware.limiter.setPosition(Values.LIMITER_CLOSED);
                    hardware.transfer.setPower(0);
                    hardware.flywheel.setPower(0);

                    follower.follow(path8());
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

            case -1:
                break;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.create(hardwareMap);
        hardware = new Hardware(hardwareMap);
        shooter = new Shooter();
        turret = new Turret();

        Values.team = Values.Team.RED;
        Values.goalX = 57.56;
        Values.goalY = 58.0;

        follower.setPose(start);
        follower.update();
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        double botX = follower.pose().x();
        double botY = follower.pose().y();
        double botHeading = follower.pose().heading();

        double targetTurretTicks = turret.Autoaim(botX, botY, botHeading, Values.goalX, Values.goalY, hardware.ll);
        turret.Turret_PID(hardware, targetTurretTicks);

        telemetry.addData("Path State", pathState);
        telemetry.addData("x", botX);
        telemetry.addData("y", botY);
        telemetry.addData("heading", Math.toDegrees(botHeading));

        if (follower.currentPath() != null) {
            telemetry.addData("Path distance remaining", follower.distanceToEndpoint());
        }

        telemetry.update();
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
