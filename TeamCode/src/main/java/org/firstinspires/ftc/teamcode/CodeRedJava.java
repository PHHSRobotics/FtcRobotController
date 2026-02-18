package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Autonomous(name = "CodeRedJava (Blocks to Java)")
public class CodeRedJava extends LinearOpMode {

    ServoV3 Feeder = new ServoV3();
/**    private Servo Feeder;*/
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor FlyWheel;
    private DcMotor HWheel;
    private DcMotor In;

    int bankVelocity;
    int maxVelocity;
    int farVelocity;

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        Feeder.init(hardwareMap);
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FlyWheel = hardwareMap.get(DcMotor.class, "FlyWheel");
        HWheel = hardwareMap.get(DcMotor.class, "HWheel");
        In = hardwareMap.get(DcMotor.class, "In");

        FlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FlyWheel.setDirection(DcMotor.Direction.REVERSE);
        HWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        HWheel.setDirection(DcMotor.Direction.REVERSE);
        bankVelocity = 1300;
        farVelocity = 1500;
        maxVelocity = 1950;
        waitForStart();
        if (opModeIsActive()) {
            DriveB();
            sleep(1600);
            StopDrive();
            In.setPower(-0.9);
            farPowerAuto();
            sleep(1850);
            StopLancher();
            TurnR();
            sleep(370);
            SRight();
            sleep(320);
            StopDrive();
            In.setPower(-1);
            DriveF();
            sleep(1650);
            StopDrive();
            DriveB();
            sleep(1500);
            StopDrive();
            TurnL();
            sleep(365);
            StopDrive();
            DriveF();
            sleep(300);
            SLeft();
            sleep(200);
            StopDrive();
            farPowerAuto();
            sleep(1000);
            StopLancher();
            DriveF();
            sleep(800);
            StopDrive();
            SRight();
            sleep(1300);
            StopDrive();
            requestOpModeStop();
        }
    }

    private void SRight() {
        BL.setPower(0.6 * -1);
        BR.setPower(0.6 * -1);
        FL.setPower(0.6 * 1);
        FR.setPower(0.6 * 1);
    }
    private void DriveF() {
        BL.setPower(0.6 * 1);
        BR.setPower(0.6 * -1);
        FL.setPower(0.6 * 1);
        FR.setPower(0.6 * -1);
    }
    private void SLeft() {
        BL.setPower(0.6 * 1);
        BR.setPower(0.6 * 1);
        FL.setPower(0.6 * -1);
        FR.setPower(0.6 * -1);
    }
    private void DriveB() {
        BL.setPower(0.6 * -1);
        BR.setPower(0.6 * 1);
        FL.setPower(0.6 * -1);
        FR.setPower(0.6 * 1);
    }
    private void TurnR() {
        BL.setPower(0.6 * 1);
        BR.setPower(0.6 * 1);
        FL.setPower(0.6 * 1);
        FR.setPower(0.6 * 1);
    }
    private void TurnL() {
        BL.setPower(0.6 * -1);
        BR.setPower(0.6 * -1);
        FL.setPower(0.6 * -1);
        FR.setPower(0.6 * -1);
    }
    private void bankShotAuto() {
        ((DcMotorEx) FlyWheel).setVelocity(bankVelocity);
        ((DcMotorEx) HWheel).setVelocity(bankVelocity);
        // servo.setPower(1); // Warning: 'servo' is not defined. Checking if it should be something else.
        if (((DcMotorEx) FlyWheel).getVelocity() >= bankVelocity - 50) {
            Feeder.setServoPos(0.3);
        } else {
            Feeder.setServoPos(0.0);
        }
    }
    private void farPowerAuto() {
        for (int count = 0; count < 300; count++) {
            ((DcMotorEx) FlyWheel).setVelocity(farVelocity);
            ((DcMotorEx) HWheel).setVelocity(farVelocity);
            // servo.setPower(1);
            if (((DcMotorEx) FlyWheel).getVelocity() >= farVelocity - 80) {
                Feeder.setServoPos(0.3);
            } else {
                Feeder.setServoPos(0.0);
            }
            telemetry.addData("Flywheel Velocity", ((DcMotorEx) FlyWheel).getVelocity());
            telemetry.addData("Flywheel Power", FlyWheel.getPower());
            telemetry.update();
        }
    }

    private void maxPowerAuto() {
        ((DcMotorEx) FlyWheel).setVelocity(maxVelocity);
        ((DcMotorEx) HWheel).setVelocity(maxVelocity);
        // servo.setPower(1);
        if (((DcMotorEx) FlyWheel).getVelocity() >= maxVelocity - 150) {
            Feeder.setServoPos(0.3);
        } else {
            Feeder.setServoPos(0.0);
        }
    }
    private void StopDrive() {
        BL.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        FR.setPower(0);
    }
    private void StopLancher() {
        FlyWheel.setPower(0);
        HWheel.setPower(0);
        Feeder.setServoPos(0.0);
    }
}
