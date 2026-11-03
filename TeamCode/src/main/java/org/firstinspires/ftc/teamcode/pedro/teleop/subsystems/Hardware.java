package org.firstinspires.ftc.teamcode.pedro.teleop.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {
    public DcMotorEx leftFront, rightFront, leftBack, rightBack;
    public DcMotorEx intake, transfer;
    public DcMotorEx flywheel;
    public Servo limiter;
    public Hardware(HardwareMap hwMap) {
        // Drive train
        leftFront = hwMap.get(DcMotorEx.class, "FL");
        leftBack = hwMap.get(DcMotorEx.class, "BL");
        rightFront = hwMap.get(DcMotorEx.class, "FR");
        rightBack = hwMap.get(DcMotorEx.class, "BR");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor[] motors = {leftFront, leftBack, rightFront, rightBack};
        for (DcMotor m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // flywheel
        flywheel = hwMap.get(DcMotorEx.class, "Flywheel1");
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // intake
        intake = hwMap.get(DcMotorEx.class, "Intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // transfer
        transfer = hwMap.get(DcMotorEx.class, "Transfer");
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // servo
        limiter = hwMap.get(Servo.class, "Limiter");
    }
}
