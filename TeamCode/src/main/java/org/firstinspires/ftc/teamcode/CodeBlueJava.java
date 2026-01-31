package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;



@Autonomous(name = "CodeBlueJava (Blocks to Java)")
public class CodeBlueJava extends LinearOpMode {

    public void init(HardwareMap hwMap) {
        Feeder = hwMap.get(Servo.class, "Feeder");
    }

    public void setPosition() {
        Feeder.setPosition(angle);
    }
    private Servo Feeder;
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor FlyWheel;
    private DcMotor In;

    int bankVelocity;
    int maxVelocity;
    int farVelocity;

    /**
     * Describe this function...
     */


    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FlyWheel = hardwareMap.get(DcMotor.class, "FlyWheel");
        In = hardwareMap.get(DcMotor.class, "In");

        FlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FlyWheel.setDirection(DcMotor.Direction.FORWARD);
        bankVelocity = 1300;
        farVelocity = 1500;
        maxVelocity = 1950;
        waitForStart();
        if (opModeIsActive()) {
            DriveB();
            sleep(1800);
            StopDrive();
            In.setPower(0.7);
            farPowerAuto();
            StopLancher();
            DriveB();
            sleep(300);
            TurnL();
            sleep(300);
            StopDrive();
            In.setPower(1);
            DriveF();
            sleep(1910);
            StopDrive();
            sleep(300);
            DriveB();
            sleep(300);
            DriveF();
            sleep(330);
            StopDrive();
            sleep(330);
            DriveB();
            sleep(1700);
            StopDrive();
            TurnR();
            sleep(370);
            DriveF();
            sleep(280);
            StopDrive();
            farPowerAuto();
            sleep(100);
            StopLancher();
            DriveF();
            sleep(400);
            StopDrive();
            SLeft();
            sleep(1200);
            StopDrive();
            requestOpModeStop();
        }
    }

    /**
     * Describe this function...
     */
    private void bankShotAuto() {
        ((DcMotorEx) FlyWheel).setVelocity(bankVelocity);
        // servo.setPower(1); // Warning: 'servo' is not defined. Checking if it should be something else.
        if (((DcMotorEx) FlyWheel).getVelocity() >= bankVelocity - 50) {
            Feeder.setPosition(0.3);
        } else {
            Feeder.setPosition(0.0);
        }
    }
    private void SRight() {
        BL.setPower(0.6 * 1);
        BR.setPower(0.6 * 1);
        FL.setPower(0.6 * -1);
        FR.setPower(0.6 * -1);
    }
    /**
     * Describe this function...
     */
    private void DriveF() {
        BL.setPower(0.6 * -1);
        BR.setPower(0.6 * 1);
        FL.setPower(0.6 * -1);
        FR.setPower(0.6 * 1);
    }

    /**
     * Describe this function...
     */
    private void SLeft() {
        BL.setPower(0.6 * -1);
        BR.setPower(0.6 * -1);
        FL.setPower(0.6 * 1);
        FR.setPower(0.6 * 1);
    }

    /**
     * Describe this function...
     */
    private void DriveB() {
        BL.setPower(0.6 * 1);
        BR.setPower(0.6 * -1);
        FL.setPower(0.6 * 1);
        FR.setPower(0.6 * -1);
    }

    /**
     * Describe this function...
     */
    private void farPowerAuto() {
        for (int count = 0; count < 300; count++) {
            ((DcMotorEx) FlyWheel).setVelocity(farVelocity);
            // servo.setPower(1);
            if (((DcMotorEx) FlyWheel).getVelocity() >= farVelocity - 100) {
                Feeder.setPosition(0.3);
            } else {
                Feeder.setPosition(0.0);
            }
            telemetry.addData("Flywheel Velocity", ((DcMotorEx) FlyWheel).getVelocity());
            telemetry.addData("Flywheel Power", FlyWheel.getPower());
            telemetry.update();
        }
    }

    /**
     * Describe this function...
     */
    private void TurnR() {
        BL.setPower(0.6 * -1);
        BR.setPower(0.6 * -1);
        FL.setPower(0.6 * -1);
        FR.setPower(0.6 * -1);
    }

    /**
     * Describe this function...
     */
    private void TurnL() {
        BL.setPower(0.6 * 1);
        BR.setPower(0.6 * 1);
        FL.setPower(0.6 * 1);
        FR.setPower(0.6 * 1);
    }

    /**
     * Describe this function...
     */
    private void maxPowerAuto() {
        ((DcMotorEx) FlyWheel).setVelocity(maxVelocity);
        // servo.setPower(1);
        if (((DcMotorEx) FlyWheel).getVelocity() >= maxVelocity - 150) {
            Feeder.setPosition(0.3);
        } else {
            Feeder.setPosition(0.0);
        }
    }

    /**
     * Describe this function...
     */
    private void StopDrive() {
        BL.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        FR.setPower(0);
    }

    /**
     * Describe this function...
     */
    private void StopLancher() {
        FlyWheel.setPower(0);
        Feeder.setPosition(0.0);
    }
}
