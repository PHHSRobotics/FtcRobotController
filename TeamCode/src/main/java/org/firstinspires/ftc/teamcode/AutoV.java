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

@Autonomous(name = "BlueVClose")
public class AutoV extends LinearOpMode {
    ServoV3 Feeder = new ServoV3();
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor FlyWheel;
    private DcMotor In;
    private IMU imu;
    private Limelight3A limelight;
    float heading_error;
    float distance_error = 1f;
    float steering_adjust = 0.0f;
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
    int step;

    boolean Correct = false;
    boolean Shoot = false;



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
        In = hardwareMap.get(DcMotor.class, "In");

        FlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FlyWheel.setDirection(DcMotor.Direction.REVERSE);
        bankVelocity = 1300;
        farVelocity = 1500;
        maxVelocity = 1950;
        waitForStart();
        if (opModeIsActive()) {

            //This is the main line of code the order of how things run
            step = 0;
            Correct = true;
            Shoot = true;

            DriveB();
            sleep(1570);
            StopDrive();


            while (Correct == true) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    tx = (float) result.getTx();
                    ty = (float) result.getTy();
                    ta = (float) result.getTa();
                    yaw = (float) result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES);
                    heading_error = -tx;
                    distance_error = ty + 9;
                } else {
                    tx = 0;
                    ty = 0;
                    ta = 0;
                    heading_error = 0;
                    distance_error = 0;
                    bond_adjust = (float) 0;
                    yaw = 0;
                    steering_adjust = 0.1f;
                }
                if (tx > 0.05) {
                    steering_adjust = (float) (KpAim * heading_error - min_aim_command - 0.2);
                } else if (tx < -0.05) {
                    steering_adjust = (float) (KpAim * heading_error + min_aim_command - 0.2);
                }

                float distance_adjust = KADistance * distance_error;

                if (yaw < -130) {
                    bond_adjust = (float) 0.7;
                } else if (yaw > -120) {
                    bond_adjust = (float) -0.7;
                } else {
                    bond_adjust = 0;
                    if (distance_error < 0.1 && distance_error > -0.1) {
                        if (tx > 0.05 && tx < -0.05) {
                            Correct = false;
                        }
                        }
                    }
                }

                float H = bond_adjust;
                float V = distance_adjust;
                float Pivot = steering_adjust;

                BL.setPower(Pivot - (H + V));
                BR.setPower(Pivot - (H - V));
                FL.setPower(Pivot + (H - V));
                FR.setPower(Pivot + (H + V));
            }

            while (Shoot == true) {
                In.setPower(-.9);
                ((DcMotorEx) FlyWheel).setVelocity(farVelocity);
                if (((DcMotorEx) FlyWheel).getVelocity() >= farVelocity - 80) {
                    Feeder.setServoPos(0.3);
                } else {
                    Feeder.setServoPos(0.0);
                }
                sleep(3000);
                Shoot = false;
                step = step + 1;
            }

            if(step == 1){
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
            }
        }
    }
    private void SRight () {
        BL.setPower(0.6 * -1);
        BR.setPower(0.6 * -1);
        FL.setPower(0.6 * 1);
        FR.setPower(0.6 * 1);
    }
    private void DriveF () {
        BL.setPower(0.6 * 1);
        BR.setPower(0.6 * -1);
        FL.setPower(0.6 * 1);
        FR.setPower(0.6 * -1);
    }
    private void SLeft () {
        BL.setPower(0.6 * 1);
        BR.setPower(0.6 * 1);
        FL.setPower(0.6 * -1);
        FR.setPower(0.6 * -1);
    }
    private void DriveB () {
        BL.setPower(0.6 * -1);
        BR.setPower(0.6 * 1);
        FL.setPower(0.6 * -1);
        FR.setPower(0.6 * 1);
    }
    private void TurnR () {
        BL.setPower(0.6 * 1);
        BR.setPower(0.6 * 1);
        FL.setPower(0.6 * 1);
        FR.setPower(0.6 * 1);
    }
    private void TurnL () {
        BL.setPower(0.6 * -1);
        BR.setPower(0.6 * -1);
        FL.setPower(0.6 * -1);
        FR.setPower(0.6 * -1);
    }

    private void StopDrive () {
        BL.setPower(0);
        BR.setPower(0);
        FL.setPower(0);
        FR.setPower(0);
    }
}
