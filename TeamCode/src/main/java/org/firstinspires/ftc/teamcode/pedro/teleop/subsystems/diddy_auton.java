package org.firstinspires.ftc.teamcode.pedro.teleop.subsystems;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PDController;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedro.teleop.Values;

@Autonomous(name = "diddy_auton", group = "Autonomous")
public class diddy_auton extends OpMode {
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
        hardware.pp.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        telemetry.update();
    }

    @Override
    public void start() {
        stopwatch.start();
        hardware.limiter.setPosition(Values.LIMITER_CLOSED);
    }

    @Override
    public void loop() {
        double vel = hardware.flywheel.getVelocity();
        if ( vel >= 1400 && stopwatch.elapsedTime() >= 5){
            hardware.limiter.setPosition(Values.LIMITER_OPEN);
            if (stopwatch.elapsedTime() >= 0.5) {
                hardware.intake.setPower(1);
                hardware.transfer.setPower(1);
            }
        }
        if (stopwatch.elapsedTime() >= 10){
          hardware.leftFront.setPower(-1.0);
          hardware.leftBack.setPower(-1.0);
          hardware.rightFront.setPower(-1.0);
          hardware.rightBack.setPower(-1.0);
        }
        double target_zero = 0;
        double pos = turret.Turret_PID(hardware, target_zero);
        double targetVel = Shooter.flywheel_Values.flywheelTarget;
        double currentVel = shooter.flywheel(hardware.flywheel, targetVel);
        telemetry.addData("Target", targetVel);
        telemetry.addData("Current", currentVel);
        telemetry.addData("time", stopwatch.elapsedTime());
        telemetry.update();

    }
}
