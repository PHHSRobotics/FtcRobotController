package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Mechanisms.ServoV3;

@TeleOp
public class ServoJavaTest extends OpMode{

    ServoV3 Feeder = new ServoV3();

    @Override
    public void init() {
        Feeder.init(hardwareMap);

    }

    @Override
    public void loop() {
        if (gamepad1.a){
            Feeder.setServoPos(0.0);
        }
        else {
            Feeder.setServoPos(1.0);
        }
    }
}

