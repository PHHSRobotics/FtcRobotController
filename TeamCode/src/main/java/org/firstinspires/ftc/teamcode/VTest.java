package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Vtest")
public class VTest extends LinearOpMode {

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


            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
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


            float H =  bond_adjust;
            float V = - steering_adjust + distance_adjust;
            float Pivot =  steering_adjust;

            BL.setPower(Pivot - (H + V));
            BR.setPower(Pivot - (H - V));
            FL.setPower(Pivot + (H - V));
            FR.setPower(Pivot + (H + V));



        }
    }

    }
