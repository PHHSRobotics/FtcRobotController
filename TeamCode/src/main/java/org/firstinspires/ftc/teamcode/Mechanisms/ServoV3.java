package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoV3 {
    private Servo Feeder;


    public void init(HardwareMap hwMap) {
        Feeder - hwMap.get(Servo.class, "Feeder");
    }

    public void setServoPos(double angle) {
        Feeder.setPosition(angle);
    }
}
