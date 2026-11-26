package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.pedro.teleop.subsystems.Hardware;

@Config
@TeleOp(name = "TurretTuner", group = "Tuning")
public class Turret_Tuner extends OpMode {

    public static double targetposition = 0;

    public static double tp = 0;
    public static double ti = 0;
    public static double td = 0;

    public static double angle = 0;

    public static double angle1 = 0;

    public static double tolerance = 50;

    public static double position = 0;


    private Servo turret1, turret2,flower,intake, limiter,hood;
    private DcMotorEx turret;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;


    PIDController pid = new PIDController(0,0,0);

    @Override
    public void init() {
        turret1 = hardwareMap.get(Servo.class, "Turret1");
        turret2 = hardwareMap.get(Servo.class, "Turret2");
        turret = hardwareMap.get(DcMotorEx.class, "Turret");
        flower = hardwareMap.get(Servo.class, "Flower");
        intake = hardwareMap.get(Servo.class,"IntakeR");
        limiter = hardwareMap.get(Servo.class, "Limiter");
        hood = hardwareMap.get(Servo.class, "Hood");

        intake.setPosition(angle1);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pid = new PIDController(tp, ti, td);
    }

    @Override
    public void loop() {
//        limiter.setPosition(range);
        limiter.setPosition(position);
        intake.setPosition(angle1);
        flower.setPosition(angle);
        double target = targetposition;
        double currentPos = turret.getCurrentPosition();
        double error = target - currentPos;

        pid.setPID(tp, ti, td);

        double power;
        if (Math.abs(error) < tolerance) {
            power = 0;
        } else {
            power = -pid.calculate(currentPos, target);
        }

        double clipped = Range.clip(power, -1.0, 1.0);
        double servoPos1 = (clipped + 1.0) / 2.0;
        double servoPos2 = servoPos1;

        turret1.setPosition(servoPos1);
        turret2.setPosition(servoPos2);

        // Telemetry
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target", target);
        packet.put("Current", currentPos);
        packet.put("Error", error);
        packet.put("Power", clipped);
        packet.put("Servo 1", servoPos1);
        packet.put("Servo 2", servoPos2);
        dashboard.sendTelemetryPacket(packet);
        telemetry.addData("angle", angle);
        telemetry.addData("Target", target);
        telemetry.addData("Current", currentPos);
        telemetry.addData("Error", error);
        telemetry.addData("Power", clipped);
        telemetry.update();
    }
}