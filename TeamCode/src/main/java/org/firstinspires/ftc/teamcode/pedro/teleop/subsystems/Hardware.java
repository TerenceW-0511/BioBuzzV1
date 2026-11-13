package org.firstinspires.ftc.teamcode.pedro.teleop.subsystems;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Hardware {
    public DcMotorEx leftFront, rightFront, leftBack, rightBack;
    public DcMotorEx intake, transfer;
    public DcMotorEx flywheel;
    public Servo limiter;

    public Servo hood;

    public Servo turret1,turret2;

    public DcMotorEx turret;
    public GoBildaPinpointDriver pp;

    public Hardware(HardwareMap hwMap) {
        /// Drive train
        leftFront = hwMap.get(DcMotorEx.class, "FL");
        leftBack = hwMap.get(DcMotorEx.class, "BL");
        rightFront = hwMap.get(DcMotorEx.class, "FR");
        rightBack = hwMap.get(DcMotorEx.class, "BR");

        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        ///

        intake = hwMap.get(DcMotorEx.class, "Intake");
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        transfer = hwMap.get(DcMotorEx.class, "Transfer");
        transfer.setDirection(DcMotorEx.Direction.REVERSE);
        transfer.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        flywheel = hwMap.get(DcMotorEx.class, "Flywheel");
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        limiter = hwMap.get(Servo.class, "Limiter");
        hood = hwMap.get(Servo.class, "Hood");

        pp = hwMap.get(GoBildaPinpointDriver.class,"pinpoint");
        turret = hwMap.get(DcMotorEx.class, "Turret");
        turret.setDirection(DcMotorEx.Direction.REVERSE);/// interchangeable
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        turret1 = hwMap.get(Servo.class, "Turret1");
        turret2 = hwMap.get(Servo.class, "Turret2");
    }
}
///Fix: Verify whether turning CCW produces positive or negative ticks. If CCW is negative, invert the target ticks returned by Autoaim (return -reqAngle * TICKS_PER_RADIAN;).
