package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TLDrive")
public class TLDrive extends LinearOpMode {

    private DcMotor BL;
    private DcMotor BR;
    private DcMotor FL;
    private DcMotor FR;
    private IMU imu;
    private Limelight3A limelight;
    float heading_error;
    float distance_error;
    float steering_adjust = 0.1f;
    float KpAim = -0.05f;
    float KpDistance = -0.1f;
    float min_aim_command = 0.05f;
    float tx;
    float ty;

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

        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                tx = (float) result.getTx();
                ty = (float) result.getTy();
                heading_error = -tx;
                distance_error = -ty;
            } else {
                tx = 0;
                ty = 0;
                heading_error = 0;
                distance_error = 0;
            }

            if (gamepad1.a) {
                requestOpModeStop();
            }

            float H = gamepad1.left_stick_x;
            float V = gamepad1.left_stick_y;
            float Pivot = gamepad1.right_stick_x;

            BL.setPower(Pivot - (H + V));
            BR.setPower(Pivot - (H - V));
            FL.setPower(Pivot + (H - V));
            FR.setPower(Pivot + (H + V));

            LLResult llResult = limelight.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                Pose3D botpose = llResult.getBotpose();
                telemetry.addData("tx", tx);
                telemetry.addData("ty", ty);
                telemetry.addData("ta", result.getTa());
                telemetry.addData("heading_error", heading_error);
                telemetry.addData("distance_error", distance_error);
                telemetry.addData("steering_adjust", steering_adjust);
                telemetry.addData("KpDistance", KpDistance);
                telemetry.addData("KpAim", KpAim);
                telemetry.addData("Botpose", botpose.toString());
                telemetry.addData("Yaw", botpose.getOrientation().getYaw(AngleUnit.DEGREES));
                telemetry.update();
            }
        }
        limelight.stop();
    }



}
