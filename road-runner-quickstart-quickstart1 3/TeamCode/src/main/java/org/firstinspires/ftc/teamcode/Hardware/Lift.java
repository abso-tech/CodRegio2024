package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {

    public CRServo servoPixel;
    public Servo servoL;

    public Servo rotation;
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
        rotation = hardwareMap.get(Servo.class,"rotation" );
        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");
        liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setDirection(DcMotorEx.Direction.REVERSE);
        servoL.setPosition(0.75);
        servoR.setPosition(0.75);
        rotation.setPosition(0.49);



    }

    public void preloadServo(){
        servoL.setPosition(0.58);
        servoR.setPosition(0.58);
    }

    public void RetractServo(){
        servoL.setPosition(0.75);
        servoR.setPosition(0.75);
    }

    public void Retract(){
        liftLeft.setTargetPosition(-100);
        liftRight.setTargetPosition(-100);

        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftLeft.setPower(1);
        liftRight.setPower(1);
    }
public void preload(){
        liftLeft.setTargetPosition(1900);
        liftRight.setTargetPosition(1900);

        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftLeft.setPower(1);
        liftRight.setPower(1);
}

    public void cycle(){
        liftLeft.setTargetPosition(2000);
        liftRight.setTargetPosition(2000);

        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftLeft.setPower(1);
        liftRight.setPower(1);
    }public void cycleHigh(){
        liftLeft.setTargetPosition(2400);
        liftRight.setTargetPosition(2400);

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
        rotation = hardwareMap.get(Servo.class,"rotation" );

        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");
        liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");

        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftRight.setDirection(DcMotorEx.Direction.REVERSE);
        servoL.setPosition(0.75);
        servoR.setPosition(0.75);
        rotation.setPosition(0.49);


    }
}