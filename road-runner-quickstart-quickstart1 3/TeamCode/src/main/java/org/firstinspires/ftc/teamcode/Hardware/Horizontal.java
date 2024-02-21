package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Horizontal {

    public DcMotorEx intakeMotor;
    public DcMotorEx intakeMotorRight;

    public Servo intakeUAD;

    public void intakeinit(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotorRight = hardwareMap.get(DcMotorEx.class, "intakeMotorRight");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeUAD = hardwareMap.get(Servo.class, "intakeUAD");
        intakeUAD.setPosition(0.2);
    }


}


