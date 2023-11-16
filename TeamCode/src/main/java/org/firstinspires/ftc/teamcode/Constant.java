package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
@Config
public class Constant {
    public static double kp = .009; //.0069
    public static double ki = 0;
    public static double kd = 0.000;

    public static double wristPos1 = 0.43; //down
    public static double wristPos2 = 0.8; //up


    public static int armPos1 = 600;
    public static int armPos2 = 650;

    public static double pixelPos1 = 0.2;
    public static double pixelPos2 = 0.5;

    public static int liftMax = 300;
    public static int liftMin = 50;


}
