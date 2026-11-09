package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.pedro.teleop.subsystems.Shooter;

@Config
@TeleOp(name = "FlywheelVelocityTuner", group = "Tuning")
public class Pid_Tuner extends OpMode {

    public static double targetVelocity = 1500;
    public static double kp=0,ki=0,kd=0,kf=0,kv=0,ks=0;
    PIDFController pidf = new PIDFController(0,0,0,0);

    private DcMotorEx flywheel;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init(){

        flywheel = hardwareMap.get(DcMotorEx.class, "Flywheel");
        flywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
    }
    @Override
    public void loop() {
        double target = targetVelocity;

        double currentVel = Math.abs(flywheel.getVelocity());
        double error = target - currentVel;
        pidf.setPIDF(kp,ki,kd,kf);

        double power = kv*target+ks+pidf.calculate(currentVel,target);

        flywheel.setPower(power);
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target Vel", target);
        packet.put("Current Vel", currentVel);
        packet.put("Error", error);
        packet.put("Power", power);
        dashboard.sendTelemetryPacket(packet);
    }
}
