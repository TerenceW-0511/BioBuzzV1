package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.teleop.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.pedro.teleop.subsystems.Shooter;

@Config
@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends OpMode {

    public Hardware hardware;
    public Follower follower;
    private Methods flywheelPID;
    private Methods methods;

    @Override
    public void init() {
        hardware = new Hardware(hardwareMap);
        flywheelPID = new Methods();
        methods = new Methods();
    }

    @Override
    public void loop() {
        follower.update();
        follower.manual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x*3/4

        );
        if (gamepad1.leftBumperWasPressed() && Values.mode== Values.Modes.SHOOTING){
            Values.init = true;
            Values.mode=Values.Modes.INTAKING;
        }
        if ((gamepad1.rightBumperWasPressed() && Values.mode == Values.Modes. INTAKING)){

            Values.init=true;
            Values.mode = Values.Modes.SHOOTING;
        }
        switch (Values.mode){
            case INTAKING:
                hardware.intake.setPower(1);
                hardware.transfer.setPower(1);
                hardware.limiter.setPosition(Values.LIMITER_CLOSED);
                break;
            case SHOOTING:
                hardware.limiter.setPosition(Values.LIMITER_OPEN);
                hardware.intake.setPower(0);
                hardware.transfer.setPower(0);
                break;
        }

        double curr = flywheelPID.flywheelFFTele(hardware.flywheel,Shooter.flywheel_Values.flywheelTarget);
    }
}
