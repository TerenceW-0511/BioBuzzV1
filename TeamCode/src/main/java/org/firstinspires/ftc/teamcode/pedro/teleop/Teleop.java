package org.firstinspires.ftc.teamcode.pedro.teleop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up && !Shooter.lastDpadUp) {
            Shooter.flywheelOffset++;
        }
        if (gamepad1.dpad_down && !Shooter.lastDpadDown) {
            Shooter.flywheelOffset--;
        }
        if (gamepad1.left_bumper) {
            hardware.intake.setPower(1);
            hardware.transfer.setPower(1);
        }else{
            hardware.intake.setPower(0);
            hardware.transfer.setPower(0);
        }
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x * 1.1;
        double rx = -gamepad1.right_stick_x;
        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        hardware.leftFront.setPower((y + x + rx) / denom);
        hardware.leftBack.setPower((y - x + rx) / denom);
        hardware.rightFront.setPower((y - x - rx) / denom);
        hardware.rightBack.setPower((y + x - rx) / denom);
        double curr = methods.flywheel(hardware.flywheel,Shooter.flywheel_Values.flywheelTarget+Shooter.flywheelOffset);

        telemetry.addData("Target", Shooter.flywheel_Values.flywheelTarget+Shooter.flywheelOffset);
        telemetry.addData("Current", curr);
//        telemetry.addData("servo pos", hardware.limiter.getPosition());
        telemetry.update();
    }

}