package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Autonomous(name = "CodeBlueJava (Blocks to Java)")
public class CodeBlueJava extends LinearOpMode {

    ServoV3 Feeder = new ServoV3();
/**    private Servo Feeder;*/
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor FlyWheel;
    //private DcMotor HWheel;
    private DcMotor In;
    private IMU imu;
    private Limelight3A limelight;
    float heading_error;
    float distance_error = 1f;
    float steering_adjust = 0.1f;
    float bond_adjust = 0f;
    float KpAim = -0.05f;
    float KpDistance = -0.1f;
    float KADistance = .3f;
    float min_aim_command = 0.05f;
    float tx;
    float ty;
    float ta;
    float yaw;

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
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        limelight.start();

        Feeder.init(hardwareMap);
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FlyWheel = hardwareMap.get(DcMotor.class, "FlyWheel");
      //  HWheel = hardwareMap.get(DcMotor.class, "HWheel");
        In = hardwareMap.get(DcMotor.class, "In");

        FlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FlyWheel.setDirection(DcMotor.Direction.REVERSE);
      //  HWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      //  HWheel.setDirection(DcMotor.Direction.REVERSE);
        bankVelocity = 1300;
        farVelocity = 1500;
        maxVelocity = 1950;
        waitForStart();
        if (opModeIsActive()) {
            DriveB();
            sleep(1570);
            StopDrive();
            //Correct();
            In.setPower(-0.9);
            farPowerAuto();
            sleep(1800);
            StopLancher();
            TurnL();
            sleep(300);
            SLeft();
            sleep(220);
            StopDrive();
            /**ball pickup*/
            In.setPower(-1);
            DriveF();
            sleep(1750);
            StopDrive();
            sleep(300);
            DriveB();
            sleep(300);
            DriveF();
            sleep(330);
            StopDrive();
            sleep(330);
            DriveB();
            sleep(1300);
            StopDrive();
            TurnR();
            sleep(370);
            DriveF();
            sleep(200);
            SRight();
            sleep(170);
            StopDrive();
            //Correct();
            farPowerAuto();
            sleep(1000);
            StopLancher();
            /** all balls shot move out of shooting area*/
            DriveF();
            sleep(500);
            StopDrive();
            SLeft();
            sleep(1200);
            StopDrive();
            limelight.stop();
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
      //  ((DcMotorEx) HWheel).setVelocity(bankVelocity);
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
          //  ((DcMotorEx) HWheel).setVelocity(farVelocity);
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
      //  ((DcMotorEx) HWheel).setVelocity(maxVelocity);
        // servo.setPower(1);
        if (((DcMotorEx) FlyWheel).getVelocity() >= maxVelocity - 150) {
            Feeder.setServoPos(0.3);
        } else {
            Feeder.setServoPos(0.0);
        }
    }
    private void Correct(){
        for (int count = 0; count < 300; count++) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid() && gamepad1.right_bumper) {
                tx = (float) result.getTx();
                ty = (float) result.getTy();
                ta = (float) result.getTa();
                yaw = (float) result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES);
                heading_error = -tx;
                distance_error = -ty - 7;
            } else {
                tx = 0;
                ty = 0;
                ta = 0;
                heading_error = 0;
                distance_error = 0;
                bond_adjust = (float) .9;
                yaw = 0;
            }
            steering_adjust = 0.0f;
            if (tx > 0.05) {
                steering_adjust = KpAim * heading_error - min_aim_command;
            } else if (tx < -0.05) {
                steering_adjust = KpAim * heading_error + min_aim_command;
            }

            float distance_adjust = KADistance * distance_error;

            if (yaw < -140) {
                bond_adjust = (float) -0.7;
            } else if (yaw > -120) {
                bond_adjust = (float) 0.7;
            }
            else {
                bond_adjust = 0;
            }


            float H = gamepad1.left_stick_x + bond_adjust;
            float V = gamepad1.left_stick_y - steering_adjust + distance_adjust;
            float Pivot = gamepad1.right_stick_x + steering_adjust;

            BL.setPower(Pivot - (H + V));
            BR.setPower(Pivot - (H - V));
            FL.setPower(Pivot + (H - V));
            FR.setPower(Pivot + (H + V));
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
      //  HWheel.setPower(0);
        Feeder.setServoPos(0.0);
    }
}
