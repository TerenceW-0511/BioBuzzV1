package org.firstinspires.ftc.teamcode.pedro.teleop;

import static org.firstinspires.ftc.teamcode.pedro.teleop.Pid_Tuner.kp;

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
import com.seattlesolvers.solverslib.controller.PIDFController;

@Config
@TeleOp(name = "TurretTuner", group = "Tuning")
public class Turret_Tuner extends OpMode {

    public static double targetposition = 0;
    public static double tp = 0;
    public static double ti = 0;
    public static double td = 0;

    public static double tf = 0;
    public static double tolerance = 20;
    public static double stableTime = 0.25;

    private Servo turret1, turret2;
    private DcMotorEx turret;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private double integral = 0;
    private double lastError = 0;

    private double stableTimer = 0;
    private ElapsedTime timer = new ElapsedTime();
    private double lastTime = 0;

    PIDFController pidf = new PIDFController(0,0,0,0);


    @Override
    public void init() {
        turret1 = hardwareMap.get(Servo.class, "Turret1");
        turret2 = hardwareMap.get(Servo.class, "Turret2");
        turret = hardwareMap.get(DcMotorEx.class, "Turret");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        timer.reset();
        lastTime = timer.seconds();
        pidf = new PIDFController(tp, ti, td, tf);
    }

    @Override
    public void loop() {
        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        if (dt <= 0) dt = 0.001;
        lastTime = currentTime;

        double target = targetposition;
        double currentPos = (turret.getCurrentPosition());

        double error = target - currentPos;

        pidf.setPIDF(tp, ti, td, tf);
        double power = pidf.calculate(currentPos, target);
        double clipped = Range.clip(power, -1.0, 1.0);
        double servoPos = (clipped + 1) / 2.0;

        turret1.setPosition(servoPos);
        turret2.setPosition(1.0 - servoPos);

        boolean inTolerance = Math.abs(error) < tolerance;

        if (inTolerance) {
            stableTimer += dt;
        } else {
            stableTimer = 0;
        }

        boolean stable = stableTimer >= stableTime;

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target Pos", target);
        packet.put("Current Pos", currentPos);
        packet.put("Error", error);
        packet.put("Servo Pos", servoPos);
        packet.put("Stable Time", stableTimer);
        packet.put("Stable?", stable);

        dashboard.sendTelemetryPacket(packet);

        telemetry.addData("Target Pos", target);
        telemetry.addData("Current Pos", currentPos);
        telemetry.addData("Error", error);
        telemetry.addData("ticks", turret.getCurrentPosition());
        telemetry.addData("Servo Pos", servoPos);
        telemetry.addData("Stable?", stable);
        telemetry.update();
    }
}