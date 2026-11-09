package org.firstinspires.ftc.teamcode.pedro.teleop;
import android.widget.Switch;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.pedro.teleop.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.pedro.teleop.subsystems.Shooter;


@TeleOp(name = "Teleop-biobust", group = "Teleop")
public class Teleop extends OpMode {
    public Hardware hardware;
    private Shooter flywheel;

    private Shooter methods;

    @Override
    public void init() {
        hardware = new Hardware(hardwareMap);
        methods = new Shooter();
        flywheel = new Shooter();
        hardware.turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hardware.turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        switch (Values.mode){
            case INTAKING:
//                hardware.intake.setPower(1);
//                hardware.transfer.setPower(1);
                hardware.limiter.setPosition(Values.LIMITER_CLOSED);
                break;
            case SHOOTING:
                hardware.intake.setPower(0);
//                hardware.transfer.setPower(1);
                hardware.limiter.setPosition(Values.LIMITER_OPEN);
                break;
        }
        if (gamepad1.dpad_left && !Shooter.lastDpadleft) {
            Shooter.hoodpos=Range.clip(Shooter.hoodpos+Shooter.HOOD_OFFSET,0.01,0.37);
        } else if (gamepad1.dpad_right && !Shooter.lastDpadright) {
            Shooter.hoodpos=Range.clip(Shooter.hoodpos-Shooter.HOOD_OFFSET,0.01,0.37);
        }
        if (gamepad1.dpad_up && !Shooter.lastDpadUp) {
            Shooter.flywheelOffset++;
        }
        if (gamepad1.dpad_down && !Shooter.lastDpadDown) {
            Shooter.flywheelOffset--;
        }
        if (gamepad1.leftBumperWasPressed() && Values.mode == Values.Modes.SHOOTING) {
            Values.mode = Values.Modes.INTAKING;
        } else if (gamepad1.rightBumperWasPressed() && Values.mode == Values.Modes.INTAKING) {
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
        double turretEncoder = hardware.turret.getCurrentPosition();
        ///
        double targetHoodPos = Range.clip(Shooter.hoodpos + Shooter.HOOD_OFFSET, 0.01, 0.37);
        hardware.hood.setPosition(targetHoodPos);
        double targetVel = Shooter.flywheel_Values.flywheelTarget + Shooter.flywheelOffset;
//        double currentVel = methods.flywheel(hardware.flywheel, targetVel);
        telemetry.addData("Turret pos", turretEncoder);
        telemetry.addData("Target", targetVel);
//        telemetry.addData("Current",currentVel);
        telemetry.addData("limiter pos", hardware.limiter.getPosition());
        telemetry.addData("hood pos", hardware.hood.getPosition());
        telemetry.addData("mode", Values.mode);
        telemetry.update();
    }
}

/// TODO LIST - limelight,hood comp?,rpm comp,autoaim,flower mech?