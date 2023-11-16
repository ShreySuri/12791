package org.firstinspires.ftc.teamcode.subSystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constant;

public class arm {
    private DcMotor arm;

    double max = 1000;

    double lowScore = 650;
    double lowScoreWrist = 0.8;
    double highScore = 560;

    double highScoreWrist = 0;
    double min = 0;
    private Servo wrist;
    private Servo pixel;

    double clearPosWrist = 0.25;
    int currPos;
    boolean ran = false;


    public arm(DcMotor arm, Servo wrist, Servo pixel){
        this.arm = arm;
        this.wrist = wrist;
        this.pixel= pixel;
        currPos = arm.getCurrentPosition();




    }

    public void goUp(int currPos){

            pixel.setPosition(.2);
            goTo(25, currPos);
            wrist.setPosition(.35);
            goTo(140, currPos);
            wrist.setPosition(.23);
            goTo(170, currPos);


        //623 - 18
        // 676 - 9
        //
    }

    public void goTo(int position, int currPos){

            arm.setPower(PID(Constant.kp,Constant.ki, Constant.kd, position, currPos));


    }

    public void goDown(){

    }



    public double PID(double Kp, double Ki, double Kd, double setpoint, double pos) {
        double integral=0, prevError=0;
        double error = setpoint - pos;
        integral += error;
        double derivative = error - prevError;
        double output = (Kp * error) + (Ki * integral) + (Kd * derivative);
        prevError = error;
        return output;
    }


}
