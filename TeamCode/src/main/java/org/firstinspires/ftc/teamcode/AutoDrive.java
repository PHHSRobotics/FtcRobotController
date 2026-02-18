package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "AutoDrive")
public class AutoDrive extends LinearOpMode {
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor FL;
    private DcMotor FR;
    private IMU imu;
    private Limelight3A limelight;
    float heading_error;
    float distance_error = 1f;
    float steering_adjust = 0.1f;
    float bond_adjust = 0f;
    float KpAim = -0.05f;
    float KpDistance = -0.05f;
    float KADistance = .09f;
    float min_aim_command = 0.05f;
    float tx;
    float ty;
    float ta;
    float yaw;

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
            if (result != null && result.isValid() && gamepad1.right_bumper) {
                tx = (float) result.getTx();
                ty = (float) result.getTy();
                ta = (float) result.getTa();
                yaw = (float) result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES);
                heading_error = -tx;
                if (ta > 1.1) {
                    distance_error = (float) -0.7;
                } else if (ta < -1.1) {
                    distance_error = (float) 0.7;
                }
                else {
                    distance_error = 0;
                }
                if (yaw > 140) {
                    bond_adjust = (float) -0.7;
                } else if (yaw < -140) {
                    bond_adjust = (float) 0.7;
                }
                else {
                    bond_adjust = 0;
                }
            } else {
                tx = 0;
                ty = 0;
                ta = 0;
                heading_error = 0;
                distance_error = 0;
                bond_adjust = 0;
            }

            if (gamepad1.a) {
                requestOpModeStop();
            }
            /*if (gamepad1.right_bumper) {**/
                steering_adjust = 0.0f;
                if (tx > 0.05) {
                    steering_adjust = KpAim * heading_error - min_aim_command;
                } else if (tx < -0.05) {
                    steering_adjust = KpAim * heading_error + min_aim_command;
                }

                float distance_adjust = KADistance * distance_error;




            float H = gamepad1.left_stick_x + bond_adjust;
            float V = gamepad1.left_stick_y - steering_adjust + distance_adjust;
            float Pivot = gamepad1.right_stick_x + steering_adjust;

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
                telemetry.addData("KADistance", KADistance);
                telemetry.addData("KpAim", KpAim);
                telemetry.addData("Botpose", botpose.toString());
                telemetry.addData("Yaw", botpose.getOrientation().getYaw(AngleUnit.DEGREES));
                telemetry.update();
            }
        }
        limelight.stop();
    }
}
