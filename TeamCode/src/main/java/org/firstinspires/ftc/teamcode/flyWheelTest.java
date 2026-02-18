package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "FlyWheelTest")
public class flyWheelTest extends LinearOpMode {
    private DcMotor FlyWheel;
    private DcMotor HWheel;

    @Override
    public void runOpMode() {
        FlyWheel = hardwareMap.get(DcMotor.class, "FlyWheel");
        HWheel = hardwareMap.get(DcMotor.class, "HWheel");

        FlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FlyWheel.setDirection(DcMotor.Direction.FORWARD);
        HWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        HWheel.setDirection(DcMotor.Direction.FORWARD);
        /**bankVelocity = 1300;
        farVelocity = 1500;
        maxVelocity = 1950;*/
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (gamepad1.a) {
                    requestOpModeStop();
                }
                runWheels();
                /**setFlywheelVelocity();
                setHWheelVelocity();*/
                telemetry.addData("Flywheel Velocity", ((DcMotorEx) FlyWheel).getVelocity());
                telemetry.addData("Flywheel Power", FlyWheel.getPower());
                telemetry.update();
            }
        }
    }

    public void runWheels() {
    if (gamepad1.dpad_down) {
        HWheel.setPower(-0.5);
    } else if (gamepad1.dpad_up) {
        HWheel.setPower(0.5);
    }else if (gamepad1.dpad_left) {
        FlyWheel.setPower(-0.5);
    }else if (gamepad1.dpad_right) {
        FlyWheel.setPower(0.5);
    }else if (gamepad1.left_bumper){
        FlyWheel.setPower(-0.5);
        HWheel.setPower(-0.5);
    }else if (gamepad1.right_bumper){
        FlyWheel.setPower(0.5);
        HWheel.setPower(0.5);
    }else{
        FlyWheel.setPower(0);
        HWheel.setPower(0);
    }
    }



}
