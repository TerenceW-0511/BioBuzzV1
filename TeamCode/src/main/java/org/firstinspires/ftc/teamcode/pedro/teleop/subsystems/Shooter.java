package org.firstinspires.ftc.teamcode.pedro.teleop.subsystems;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;

@Config
@Configurable
public class Shooter {

    public static class flywheel_Values {
        public static double fP = 0.0045;
        public static double fI = 0;
        public static double fD = 0.0000001;
        public static double fF = 0;
        public static double fV = 0.000364;
        public static double fS = 0.04;
        public static PIDFController flywheelPIDController = new PIDFController(fP, fI, fD, fF);

        public static double kV = 0;
        public static double kS = 0;


        public static double flywheelTarget=3300;
        public static double flywheelIdle = 1500; //MAX 2300

    }
}
