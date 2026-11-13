package org.firstinspires.ftc.teamcode.pedro.teleop;
import android.widget.Switch;

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
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.pedro.teleop.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.pedro.teleop.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedro.teleop.subsystems.Turret;

@TeleOp(name = "Teleop-biobust", group = "Teleop")
public class Teleop extends OpMode {
    public Hardware hardware;
    private Shooter flywheel;

    private Shooter shooter;
    private Timing.Stopwatch stopwatch;

    private Turret turret;

    @Override
    public void init() {
        hardware = new Hardware(hardwareMap);
        shooter = new Shooter();
        flywheel = new Shooter();
        turret = new Turret();
        stopwatch = new Timing.Stopwatch();
        hardware.turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hardware.turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        hardware.hood.setPosition(Values.HOOD_MAX);
    }

    @Override
    public void start() {
        stopwatch.start();
        hardware.pp.update();
        hardware.pp.resetPosAndIMU();
        hardware.pp.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }

    @Override
    public void loop() {
        hardware.pp.update();
        double botx = hardware.pp.getPosX(DistanceUnit.INCH);
        double boty = hardware.pp.getPosY(DistanceUnit.INCH);
        double botheading = hardware.pp.getHeading(AngleUnit.RADIANS);
        double targetTurretTicks = turret.Autoaim(
                botx,
                boty,
                botheading,
                Values.goalX,
                Values.goalY
        );


        switch (Values.mode) {
            case INTAKING:
                hardware.intake.setPower(gamepad1.circle ? 1 : 0);
                hardware.transfer.setPower(1);
                hardware.limiter.setPosition(Values.LIMITER_CLOSED);
                break;
            case SHOOTING:
                hardware.intake.setPower(gamepad1.circle ? 1 : 0);
                hardware.limiter.setPosition(Values.LIMITER_OPEN);
                break;
        }
        if (gamepad1.dpad_left && !Shooter.lastDpadleft) {
            Shooter.hoodpos = Range.clip(Shooter.hoodpos + Shooter.HOOD_OFFSET, 0.01, 0.37);
        } else if (gamepad1.dpad_right && !Shooter.lastDpadright) {
            Shooter.hoodpos = Range.clip(Shooter.hoodpos - Shooter.HOOD_OFFSET, 0.01, 0.37);
        }
        if (gamepad1.dpad_up && !Shooter.lastDpadUp) {
            Shooter.flywheelOffset++;
        }
        if (gamepad1.dpad_down && !Shooter.lastDpadDown) {
            Shooter.flywheelOffset--;
        }
        if (gamepad1.leftBumperWasPressed() && Values.mode == Values.Modes.SHOOTING) {
            Values.mode = Values.Modes.INTAKING;
            stopwatch.start();
        } else if (gamepad1.rightBumperWasPressed() && Values.mode == Values.Modes.INTAKING && stopwatch.elapsedTime() > 0.5) {
            Values.mode = Values.Modes.SHOOTING;
        }

            Shooter.lastDpadleft = gamepad1.dpad_left;
            Shooter.lastDpadright = gamepad1.dpad_right;
            Shooter.lastDpadUp = gamepad1.dpad_up;
            Shooter.lastDpadDown = gamepad1.dpad_down;
            /// teleop dt
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            double rx = -gamepad1.right_stick_x;
            double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

            hardware.leftFront.setPower((y + x + rx) / denom);
            hardware.leftBack.setPower((y - x + rx) / denom);
            hardware.rightFront.setPower((y - x - rx) / denom);
            hardware.rightBack.setPower((y + x - rx) / denom);
            ///
            double targetHoodPos = Range.clip(Shooter.hoodpos + Shooter.HOOD_OFFSET, 0.01, 0.37);
            hardware.hood.setPosition(targetHoodPos);
            double targetVel = Shooter.flywheel_Values.flywheelTarget + Shooter.flywheelOffset;
            double currentVel = shooter.flywheel(hardware.flywheel, targetVel);

//            double pos = turret.Turret_PID(hardware, targetTurretTicks);
            double turretEncoder = hardware.turret.getCurrentPosition();
        ///flywheel
            telemetry.addData("Target", targetVel);
            telemetry.addData("Current", currentVel);
            telemetry.addData("limiter pos", hardware.limiter.getPosition());
            telemetry.addData("hood pos", hardware.hood.getPosition());
            telemetry.addData("mode", Values.mode);
            ///turret
            telemetry.addData("heading", Turret.ticksToDegrees(botheading));
            telemetry.addData("bearing heading", Turret.ticksToDegrees(turret.Autoaim(botx, boty, botheading, Values.goalX, Values.goalY)));
            telemetry.addData("target turret",targetTurretTicks);
            telemetry.addData("Turret current", turretEncoder);
//            telemetry.addData("Turret pos", pos);
            telemetry.addData("x", botx);
            telemetry.addData("y", boty);
            telemetry.update();

        }
    }

/// TODO LIST - limelight,hood comp?,rpm comp,autoaim,flower mech?