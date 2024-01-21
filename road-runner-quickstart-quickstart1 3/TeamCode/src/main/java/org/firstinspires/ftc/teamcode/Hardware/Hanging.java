package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hanging {
 public Servo LeftHang;
public Servo RightHang;
public void initHang(HardwareMap hardwareMap) {

    LeftHang = hardwareMap.get(Servo.class, "LeftHang");
    RightHang= hardwareMap.get(Servo.class, "RightHang");
    LeftHang.setPosition(1);
    RightHang.setPosition(1);
}
}
