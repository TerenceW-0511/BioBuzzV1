package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.pedro.teleop.subsystems.Hardware;
import org.firstinspires.ftc.teamcode.pedro.teleop.subsystems.Shooter;

public class Methods {

    public double flywheelFFTele(DcMotorEx m1, double target) {
        double v1 = m1.getVelocity();
        boolean v1Dead = Math.abs(v1) < 50;
        double curr;
        if (v1Dead) {
            curr = v1;
        }
        double power = Shooter.flywheel_Values.fV * target + Shooter.flywheel_Values.fS + Shooter.flywheel_Values.flywheelPIDController.calculate(v1, target);
        m1.setPower(power);
        return v1;
    }
}