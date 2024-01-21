package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {

    public CRServo servoPixel;
    public Servo servoL;
    public Servo servoR;
    public DcMotorEx liftLeft;
    public DcMotorEx liftRight;


    public double outTakeUp = 0.05;
    public double outTakeDown = 0.23;
    public int primaLinie = 100;
    public int aDouaLinie = 210;
    public int aTreiaLinie = 480;

    public void initLiftAuto(HardwareMap hardwareMap) {

        servoPixel = hardwareMap.get(CRServo.class,"servoPixel" );
        servoL = hardwareMap.get(Servo.class,"servoL" );
        servoR = hardwareMap.get(Servo.class,"servoR" );
        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");
        liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setDirection(DcMotorEx.Direction.REVERSE);
        servoL.setPosition(0.828);
        servoR.setPosition(0.828);



    }

    public void preloadServo(){
        servoL.setPosition(0.62);
        servoR.setPosition(0.62);
    }

    public void RetractServo(){
        servoL.setPosition(0.812);
        servoR.setPosition(0.812);
    }

    public void Retract(){
        liftLeft.setTargetPosition(-10);
        liftRight.setTargetPosition(-10);

        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftLeft.setPower(1);
        liftRight.setPower(1);
    }
public void preload(){
        liftLeft.setTargetPosition(270);
        liftRight.setTargetPosition(270);

        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftLeft.setPower(1);
        liftRight.setPower(1);
}
    public void cycle(){
        liftLeft.setTargetPosition(350);
        liftRight.setTargetPosition(350);

        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftLeft.setPower(1);
        liftRight.setPower(1);
    }
    public void Low(){
        servoL.setPosition(0.05);
        servoR.setPosition(0.05);
        liftLeft.setTargetPosition(100);
        liftRight.setTargetPosition(100);
    }

    public void Medium(){
        servoL.setPosition(0.05);
        servoR.setPosition(0.05);
        liftLeft.setTargetPosition(210);
        liftRight.setTargetPosition(210);
    }

    public void High(){
        servoL.setPosition(0.05);
        servoR.setPosition(0.05);
        liftLeft.setTargetPosition(480);
        liftRight.setTargetPosition(480);
    }

    public void DownLift(){
        servoL.setPosition(0.23);
        servoR.setPosition(0.23);
        liftLeft.setTargetPosition(0);
        liftRight.setTargetPosition(0);
    }

    public void initLiftTeleOp(HardwareMap hardwareMap) {

        servoPixel = hardwareMap.get(CRServo.class,"servoPixel" );
        servoL = hardwareMap.get(Servo.class,"servoL" );
        servoR = hardwareMap.get(Servo.class,"servoR" );
        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");
        liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");

        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftRight.setDirection(DcMotorEx.Direction.REVERSE);
        servoL.setPosition(0.828);
        servoR.setPosition(0.828);


    }
}