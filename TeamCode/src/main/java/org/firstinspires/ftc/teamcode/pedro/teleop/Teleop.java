package org.firstinspires.ftc.teamcode.pedro.teleop;
import android.widget.Switch;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.util.Timing;
import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.teleop.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.pedro.teleop.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedro.teleop.subsystems.Turret;

@TeleOp(name = "Teleop-biobust", group = "Teleop")
public class Teleop extends OpMode {
    public Hardware hardware;
    private Shooter flywheel;
    double target_zero = 0;
    private Shooter shooter;
    private Timing.Stopwatch stopwatch;

    private Follower robot;
    private Turret turret;

    @Override
    public void init() {
        hardware = new Hardware(hardwareMap);
        shooter = new Shooter();
        flywheel = new Shooter();
        turret = new Turret();
        robot = Constants.create(hardwareMap);
        stopwatch = new Timing.Stopwatch(TimeUnit.MILLISECONDS);
        hardware.turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hardware.turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        hardware.hood.setPosition(Values.HOOD_MAX);
        double pos = turret.Turret_PID(hardware, target_zero);
        telemetry.update();
        hardware.sintake.setPosition(Values.angle);
        robot.update();
        robot.setPose(new Pose(8.1765, 5.9465, Math.toRadians(0)));
        Values.farGoal = false;
    }

    @Override
    public void init_loop() {
        telemetry.addData("team :",Values.team);
        telemetry.addData("angle :",Values.flowerRampOff);

        if (gamepad1.leftBumperWasPressed()) {
            Values.team = Values.Team.BLUE;
        } else if (gamepad1.rightBumperWasPressed()) {
            Values.team = Values.Team.RED;
        }
    }
    @Override
    public void start() {
        robot.manual();
        stopwatch.start();
        hardware.pp.update();
//        if (Values.team != lastTeam) {
//            hardware.ll.pipelineSwitch(Values.team == Values.Team.RED ? 1 : 2);
//            lastTeam = Values.team;
//        }
        hardware.ll.start();
        hardware.ll.pipelineSwitch(Values.team == Values.Team.RED ? 1 : 2);
    }


    @Override
    public void loop() {
        hardware.sintake.setPosition(Values.angle);
        hardware.flower.setPosition(Values.flowerRampOff);
        robot.update();
//        if (gamepad1.startWasPressed) {
//            Pose cornerPose = new Pose(10.5, 10.5, Math.toRadians(90));
//            // On the fly Pose creation, we dont recommend this for Autonomous. Only accepts radians for heading
//            follower.setPose(cornerPose); // overrides our pose
//        }
        double botx = robot.pose().x();
        double boty = robot.pose().y();
        double botheading = robot.pose().heading();
//        double botx = hardware.pp.getPosX(DistanceUnit.INCH);
//        double boty = hardware.pp.getPosY(DistanceUnit.INCH);
//        double botheading = hardware.pp.getHeading(AngleUnit.RADIANS);
        if (!Values.farGoal && boty > Values.FAR_ON) {
            Values.farGoal = true;
        } else if (Values.farGoal && boty < Values.FAR_OFF) {
            Values.farGoal = false;
        }
        if (Values.team == Values.Team.BLUE) {
            Values.goalX = 84.0;
            if (Values.farGoal) {
                Values.goalY =  83.0;
            } else {
                Values.goalY =  52.0;
            }

        } else {
            Values.goalX = 57.56;
            if (Values.farGoal) {
                Values.goalY =  88.0;
            } else {
                Values.goalY =  58.0;
            }
        }
        robot.manual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );
        double targetTurretTicks = turret.Autoaim(
                botx,
                boty,
                botheading,
                Values.goalX,
                Values.goalY,
                hardware.ll
        );
        if (gamepad1.circle){
            hardware.pp.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        }
        if (gamepad1.x){
            Values.tx=0;

            hardware.ll.pause();
            hardware.ll.start();

            Shooter.manualRelocalize(robot);
        }



        switch (Values.mode) {
            case INTAKING:
//                hardware.transfer.setPower(gamepad1.circle ? 1 : 0);
//                hardware.intake.setPower(gamepad1.circle ? 1 : 0);
                hardware.limiter.setPosition(Values.LIMITER_CLOSED);
                stopwatch.start();
                break;
            case SHOOTING:
                hardware.limiter.setPosition(Values.LIMITER_OPEN);
                if (stopwatch.elapsedTime() > 200) {
                    hardware.intake.setPower(1);
                    hardware.transfer.setPower(1);
                }
                break;
        }
//        if (Math.abs(gamepad2.left_stick_x) > 0.01) {
//            double manualPower = gamepad2.left_stick_x;
//            double servoPos1 = (manualPower + 1.0) / 2.0;
//            double servoPos2 = servoPos1;
//
//            hardware.turret1.setPosition(servoPos1);
//            hardware.turret2.setPosition(servoPos2);
//        }
        if (gamepad1.dpad_left && !Shooter.lastDpadleft) {
            Shooter.hoodpos = Range.clip(Shooter.hoodpos + Shooter.HOOD_OFFSET, 0.01, 0.37);
        } else if (gamepad1.dpad_right && !Shooter.lastDpadright) {
            Shooter.hoodpos = Range.clip(Shooter.hoodpos - Shooter.HOOD_OFFSET, 0.01, 0.37);
        }
        if (gamepad1.dpad_up && !Shooter.lastDpadUp) {
            Shooter.flywheelOffset = Shooter.flywheelOffset+Shooter.SHOOTER_OFFSET;
//            angle += 0.01;
        }
        if (gamepad1.dpad_down && !Shooter.lastDpadDown) {
            Shooter.flywheelOffset = Shooter.flywheelOffset-Shooter.SHOOTER_OFFSET;
//            angle -= 0.01;
        }
        if (gamepad1.leftBumperWasPressed() && Values.mode == Values.Modes.SHOOTING) {
            Values.mode = Values.Modes.INTAKING;
        } else if (gamepad1.rightBumperWasPressed() && Values.mode == Values.Modes.INTAKING) {
            Values.mode = Values.Modes.SHOOTING;
        }
        if (gamepad1.left_bumper) {
            hardware.intake.setPower(1);
            hardware.transfer.setPower(1);
        } else if (Values.mode != Values.Modes.SHOOTING) {
            hardware.transfer.setPower(0);
            hardware.intake.setPower(0);
        }


            Shooter.lastDpadleft = gamepad1.dpad_left;
            Shooter.lastDpadright = gamepad1.dpad_right;
            Shooter.lastDpadUp = gamepad1.dpad_up;
            Shooter.lastDpadDown = gamepad1.dpad_down;
            /// teleop dt

        //            double y = gamepad1.left_stick_y;
//            double x = -gamepad1.left_stick_x * 1.1;
//            double rx = -gamepad1.right_stick_x;
//            double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
//
//            hardware.leftFront.setPower((y + x + rx) / denom);
//            hardware.leftBack.setPower((y - x + rx) / denom);
//            hardware.rightFront.setPower((y - x - rx) / denom);
//            hardware.rightBack.setPower((y + x - rx) / denom);
            ///
            double targetHoodPos = Range.clip(Shooter.hoodpos + Shooter.HOOD_OFFSET, 0.01, 0.37);
            hardware.hood.setPosition(targetHoodPos);
            double targetRPM = Shooter.rpmComp(Turret.Distance(botx, boty, Values.goalX, Values.goalY));
            double targetVel = targetRPM + Shooter.flywheelOffset;
//            double targetVel = Shooter.flywheel_Values.flywheelTarget + Shooter.flywheelOffset;
            double currentVel = shooter.flywheel(hardware.flywheel, targetVel);
            double target_zero = 0;
//            double pos = turret.Turret_PID(hardware, target_zero);
            double pos = turret.Turret_PID(hardware, targetTurretTicks);
            double turretEncoder = hardware.turret.getCurrentPosition();
        ///flywheel
            telemetry.addData("connection info",hardware.ll.getConnectionInfo());
            telemetry.addData("servo val", Values.angle);
            telemetry.addData("Target", targetVel);
            telemetry.addData("Current", currentVel);
            telemetry.addData("limiter pos", hardware.limiter.getPosition());
            telemetry.addData("hood pos", hardware.hood.getPosition());
            telemetry.addData("mode", Values.mode);
            telemetry.addData("dist",Turret.Distance(botx, boty, Values.goalX, Values.goalY));
            ///turret
            telemetry.addData("target turret",targetTurretTicks);
            telemetry.addData("Turret current", turretEncoder);
            telemetry.addData("Turret pos", pos);
            telemetry.addData("heading",(botheading));
            telemetry.addData("x", botx);
            telemetry.addData("y", boty);
            telemetry.addData("robot pose", robot.pose());
//            telemetry.addData("connection info",hardware.ll.getConnectionInfo());

        telemetry.update();

        }
    }
///tmrw check Telemetry robot.getPose() and hardware.pp.getPosX/Y/Heading .
/// also Spin 360° in place if x,y within ~1 in of start and If  ~9 in, offsets are wrong
/// TODO LIST - limelight,hood comp?,rpm comp,autoaim,flower mech?
/// THE NEW SERVOS ARE IntakeR IntakeL Flower
