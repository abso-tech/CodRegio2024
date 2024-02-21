package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Airplane {
public Servo AirplaneUAD;
public Servo AirplaneLaunch;


 public void initAirplane(HardwareMap hardwareMap) {
     AirplaneUAD = hardwareMap.get(Servo.class, "AirplaneUAD");
     AirplaneLaunch = hardwareMap.get(Servo.class, "AirplaneLaunch");
     AirplaneUAD.setPosition(0.2);
     AirplaneLaunch.setPosition(0.5);
 }

}
