package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subSystem.*;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;

    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    DcMotor armMotor;
    DcMotor intakeMotor;
    DcMotor droneMotor;
    Servo wristServo;
    Servo pixelLockServo;

    double power;
    double slowMode;





    double lastKp = Constant.kp;
    double lastKi = Constant.ki;
    double lastKd = Constant.kd;
    double wPos1 = .5;
    double wPos2 = .5;
    double pPos1 = .5;
    double pPos2 = .5;


    WebcamName camera = hardwareMap.get(WebcamName.class, "Webcam 1");

    TfodProcessor processor = TfodProcessor.easyCreateWithDefaults();
    //VisionPortal vp = VisionPortal.easyCreateWithDefaults(camera, visionProcessors);
// to be implemented


    public void runOpMode() throws InterruptedException {



        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();






        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        droneMotor = hardwareMap.dcMotor.get("droneMotor");

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        droneMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wristServo = hardwareMap.servo.get("wristServo");
        pixelLockServo = hardwareMap.servo.get("pixelLockServo");

        intakeMotor.setTargetPosition(0);

        power = .4;
        slowMode = 0.3;


        armMotor.resetDeviceConfigurationForOpMode();
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int incrementArm = armMotor.getCurrentPosition();

        boolean isLocked = true;
        boolean prevY = false;
        boolean prevX = false;
        boolean prevDpad = false;
        int ARM_DOWN = 0;
        int ARM_UP = 0;
        double intakePower = 0;
        int intakeMode = 0;
        double wristSpeed = 0.00;
        double dronePower = 1;
        boolean droneLaunched = false;


        double x;
        double y;
        double rx;

        arm stuff = new arm(armMotor, wristServo, pixelLockServo);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {


            x = gamepad1.left_stick_x;
            y = gamepad1.left_stick_y ;
            rx = gamepad1.right_stick_x  ;
            updateTrajectory(x, y, rx);

            if(gamepad1.a)
            {
                incrementArm+=5;
            }
            if(gamepad1.b)
            {
                incrementArm-=5;
            }
            if(gamepad1.right_bumper)
            {
                wristServo.setPosition(wPos2); //down
            }
            if(gamepad1.right_trigger > .1)
            {
                pixelLockServo.setPosition(pPos1);
            }


            if(gamepad1.dpad_down){
                intakeMotor.setPower(.8);
            }
            if(gamepad1.dpad_up){
                intakeMotor.setPower(-.8);
            }
            if(gamepad1.dpad_right){
                intakeMotor.setPower(0);
            }
            if(gamepad1.y){
                wristServo.setPosition(.25);
            }
            if(gamepad1.x) {
                incrementArm = 620;
            }
            if(gamepad1.left_bumper)
            {
               wristServo.setPosition(wPos1);
            }
            if(gamepad1.left_trigger > .1)
            {
                pixelLockServo.setPosition(pPos2);
            }


//            if(gamepad1.left_bumper && gamepad1.right_bumper && !droneLaunched)
//            {
//                droneMotor.setPower(dronePower);
//                sleep(100);
//                droneMotor.setPower(0);
//                droneLaunched = true;
//
//
//            }


            if (lastKp != Constant.kp || lastKd != Constant.kd
                    || lastKi != Constant.kd ) {

                lastKp = Constant.kp;
                lastKi = Constant.kd;
                lastKd = Constant.kd;
            }

            if (wPos1 != Constant.wristPos1 || pPos1 != Constant.pixelPos1 || wPos2 != Constant.wristPos2 || pPos2 != Constant.pixelPos2) {

                wPos1 = Constant.wristPos1;
                pPos1 = Constant.pixelPos1;
                wPos2 = Constant.wristPos2;
                pPos2 = Constant.pixelPos2;
            }


            if(incrementArm > 570) wristServo.setPosition(servoScorePos(armMotor.getCurrentPosition()));
            //if(incrementArm < 200) wristServo.setPosition(servoExitPos(armMotor.getCurrentPosition()));

            armMotor.setPower(PID(lastKp,lastKi,lastKd, incrementArm, armMotor.getCurrentPosition()));

            telemetry.addData("armPos", armMotor.getCurrentPosition());

            telemetry.update();

        }

        telemetry.update();
    }

    private double servoScorePos(int pos) {


        int armEncoderMin = 570;
        int armEncoderMax = 770;
        int armEncoderPoint1 = 671;
        int armEncoderPoint2 = 725;
        double servoPosPoint1 = .90;
        double servoPosPoint2 = 0.85;

        double servoPos = servoPosPoint1 + (servoPosPoint2 - servoPosPoint1) * (pos - armEncoderPoint1) / (armEncoderPoint2 - armEncoderPoint1);

        return Math.max(0.0, Math.min(1.0, servoPos));

    }


    private double servoExitPos(int pos) {
        int armEncoderMin = -10;
        int armEncoderMax = 200;

        int armEncoderPoint1 = 0;
        int armEncoderPoint2 = 28;
        int armEncoderPoint3 = 166;
        int armEncoderPoint4 = 115;

        double servoPosPoint1 = 0.43;
        double servoPosPoint2 = 0.35;
        double servoPosPoint3 = 0.27;
        double servoPosPoint4 = 0.21;

        double servoPosSegment1 = servoPosPoint1 + (servoPosPoint2 - servoPosPoint1) * (pos - armEncoderPoint1) / (armEncoderPoint2 - armEncoderPoint1);
        double servoPosSegment2 = servoPosPoint3 + (servoPosPoint4 - servoPosPoint3) * (pos - armEncoderPoint3) / (armEncoderPoint4 - armEncoderPoint3);
        double servoPos = servoPosSegment1 + (servoPosSegment2 - servoPosSegment1) * (pos - armEncoderPoint1) / (armEncoderPoint4 - armEncoderPoint1);

        return Math.max(0.0, Math.min(1.0, servoPos));
    }


    public void updateTrajectory(double x, double y, double rx){

            if (x == 0 && y == 0 && rx == 0 && power == slowMode) {
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
            } else {
                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (-x + y - rx) / denominator;
                double backLeftPower = (-x - y + rx) / denominator;
                double frontRightPower = (x + y + rx) / denominator;
                double backRightPower = (x - y - rx) / denominator;

                frontLeftMotor.setPower(frontLeftPower * power);
                backLeftMotor.setPower(backLeftPower * power);
                frontRightMotor.setPower(frontRightPower * power);
                backRightMotor.setPower(backRightPower * power);
            }



    }


    public double PID(double Kp, double Ki, double Kd, double setpo1nt, double pos) {
        double integral=0, prevError=0;
        double error = setpo1nt - pos;
        integral += error;
        double derivative = error - prevError;
        double output = (Kp * error) + (Ki * integral) + (Kd * derivative);
        prevError = error;
        return output;
    }

    public enum liftState {
        START,
        EXTEND,
        DUMP,
        RETRACT
    }



}
