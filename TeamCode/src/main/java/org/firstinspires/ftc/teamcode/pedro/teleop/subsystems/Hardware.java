package org.firstinspires.ftc.teamcode.pedro.teleop.subsystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Hardware {
    public DcMotorEx leftFront, rightFront, leftBack, rightBack;
    public DcMotorEx intake, transfer;
    public DcMotorEx flywheel;
    public Servo limiter;
    public Servo flower; // not on the robot and never mapped. Only here so Red_auton still builds, delete once Red_auton stops using it

    public Servo sintake;

    public Limelight3A ll;
    public Servo hood;

    public Servo turret1,turret2;

    public DcMotorEx turret;
    public GoBildaPinpointDriver pp;

//    / Autos and tuners: zeros the turret encoder, so the turret has to point straight forward at INIT. */
    public Hardware(HardwareMap hwMap) {
        this(hwMap, true);
    }

//    zeroTurret = false keeps the encoder count from the last OpMode (teleop after auto),
//    so the turret's zero stays at real center instead of wherever auto left it.*/
    public Hardware(HardwareMap hwMap, boolean zeroTurret) {/// Drive train
        leftFront = hwMap.get(DcMotorEx.class, "FL");
        leftBack = hwMap.get(DcMotorEx.class, "BL");
        rightFront = hwMap.get(DcMotorEx.class, "FR");
        rightBack = hwMap.get(DcMotorEx.class, "BR");

//        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
//        rightBack.setDirection(DcMotorEx.Direction.REVERSE);

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
        flywheel.setDirection(DcMotorEx.Direction.FORWARD);
        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        limiter = hwMap.get(Servo.class, "Limiter");
        hood = hwMap.get(Servo.class, "Hood");

        pp = hwMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        sintake = hwMap.get(Servo.class, "IntakeR");
        turret = hwMap.get(DcMotorEx.class, "Turret");
        turret.setDirection(DcMotorEx.Direction.REVERSE);/// interchangeable
        if (zeroTurret) {
            turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        turret1 = hwMap.get(Servo.class, "Turret1");
        turret2 = hwMap.get(Servo.class, "Turret2");

        ll = hwMap.get(Limelight3A.class, "ll");
    }
}
