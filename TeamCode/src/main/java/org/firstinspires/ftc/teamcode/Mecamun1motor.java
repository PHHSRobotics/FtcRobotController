package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Mecanum1Motor (Blocks to Java)")
public class Mecamun1motor extends LinearOpMode {
/** this is fun*/
  private DcMotor In;
  private DcMotor FlyWheel;
 // private DcMotor HWheel;
  private DcMotor BL;
  private DcMotor BR;
  private DcMotor FL;
  private DcMotor FR;
  private Servo Feeder;
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
  int maxVelocity;
  int bankVelocity;
  int farVelocity;
  double theVelocity;
  private double distance;
  double scale = 4824.53203;

  /**
   * Describe this function...
   */

  public double getDistance() {
      YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
      limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
      LLResult result = limelight.getLatestResult();
      ta = (float) result.getTa();
      double distance = (scale / ta);
      return distance;
  }
  private void manualCoreHexAndServoControl() {
    // Manual control for the Core Hex feeder

    if (gamepad1.dpad_down) {
        Feeder.setPosition(0.3);
    } else if (gamepad1.dpad_up) {
        Feeder.setPosition(0.0);
    }
    // Manual control for the hopper's servo
    if (gamepad1.right_stick_button) {
      In.setPower(0.5);
    } else if (gamepad1.left_stick_button) {
      In.setPower(-1);
    } else if (gamepad1.circle) {
      In.setPower(0);
    }
  }

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

    float H;
    float V;
    float Pivot;
    Feeder = hardwareMap.get(Servo.class, "Feeder");
    In = hardwareMap.get(DcMotor.class, "In");
    FlyWheel = hardwareMap.get(DcMotor.class, "FlyWheel");
    //HWheel = hardwareMap.get(DcMotor.class, "HWheel");
    BL = hardwareMap.get(DcMotor.class, "BL");
    BR = hardwareMap.get(DcMotor.class, "BR");
    FL = hardwareMap.get(DcMotor.class, "FL");
    FR = hardwareMap.get(DcMotor.class, "FR");

    FlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    FlyWheel.setDirection(DcMotor.Direction.REVERSE);
    //HWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //HWheel.setDirection(DcMotor.Direction.REVERSE);
    Feeder.setPosition(0.0);
    bankVelocity = 1300;
    farVelocity = 1500;
    maxVelocity = 1950;


    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
          theVelocity = (1447.5/ta);
          LLResult result = limelight.getLatestResult();
          if (result != null && result.isValid()) {
              if (gamepad1.right_bumper || gamepad1.left_bumper || gamepad1.triangle) {
                  tx = (float) result.getTx();
                  ty = (float) result.getTy();
                  ta = (float) result.getTa();
                  yaw = (float) result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES);
                  heading_error = -tx;
                  distance_error = ty;

              }
          } else {
              tx = 0;
              ty = 0;
              ta = 0;
              heading_error = 0;
              distance_error = 0;
              yaw = 0;
              theVelocity = 0;
          }

          steering_adjust = 0.0f;
          if (tx > 0.05) {
              steering_adjust = (float) (KpAim * heading_error - min_aim_command - 0.2);
          } else if (tx < -0.05) {
              steering_adjust = (float) (KpAim * heading_error + min_aim_command - 0.2);
          }

          float distance_adjust = KADistance * distance_error;

          if (yaw > 140) {
              bond_adjust = (float) -0.7;
          } else if (yaw < -140) {
              bond_adjust = (float) 0.7;
          } else {
              bond_adjust = 0;
          }

          if (gamepad1.a) {
              requestOpModeStop();
          }
          manualCoreHexAndServoControl();
          setFlywheelVelocity();
          H = gamepad1.left_stick_x;
          V = gamepad1.left_stick_y;
          Pivot = gamepad1.right_stick_x + steering_adjust;
          BL.setPower(1 * (Pivot - (H + V)));
          BR.setPower(1 * (Pivot - (H - V)));
          FL.setPower(1 * (Pivot + (H - V)));
          FR.setPower(1 * (Pivot + (H + V)));

          LLResult llResult = limelight.getLatestResult();
          if (llResult != null && llResult.isValid()) {
              Pose3D botpose = llResult.getBotpose();
              distance = getDistance();
              telemetry.addData("tx", tx);
              telemetry.addData("ty", ty);
              telemetry.addData("ta", result.getTa());
              telemetry.addData("Distance", distance);
              telemetry.addData("heading_error", heading_error);
              telemetry.addData("distance_error", distance_error);
              telemetry.addData("steering_adjust", steering_adjust);
              telemetry.addData("KpDistance", KpDistance);
              telemetry.addData("KADistance", KADistance);
              telemetry.addData("KpAim", KpAim);
              telemetry.addData("Yaw", botpose.getOrientation().getYaw(AngleUnit.DEGREES));
          }
        telemetry.addData("theVelocity", theVelocity);
        //telemetry.addData("HWheel Velocity", ((DcMotorEx) HWheel).getVelocity());
        //telemetry.addData("HWheel Power", HWheel.getPower());
        telemetry.addData("Flywheel Velocity", ((DcMotorEx) FlyWheel).getVelocity());
        telemetry.addData("Flywheel Power", FlyWheel.getPower());
        telemetry.addData("Distance", distance);
        telemetry.update();
      }
    }
  }

  /**
   * Describe this function...
   */
  private void maxPowerAuto() {
    ((DcMotorEx) FlyWheel).setVelocity(maxVelocity);
    //((DcMotorEx) HWheel).setVelocity(maxVelocity);
    In.setPower(-.9);
    if (((DcMotorEx) FlyWheel).getVelocity() >= maxVelocity - 140) {
        Feeder.setPosition(0.2);
    } else {
        Feeder.setPosition(0.0);
    }
  }

  /**
   * Describe this function...
   */
  private void bankShotAuto() {
    ((DcMotorEx) FlyWheel).setVelocity(bankVelocity);
    //((DcMotorEx) HWheel).setVelocity(bankVelocity);
    In.setPower(-1);
    if (((DcMotorEx) FlyWheel).getVelocity() >= bankVelocity - 50) {
        Feeder.setPosition(0.2);
    } else {
        Feeder.setPosition(0.0);
    }
  }

  /**
   * Describe this function...
   */
  private void setFlywheelVelocity() {
    if (gamepad1.options) {
      FlyWheel.setPower(-0.5);
      //HWheel.setPower(-0.5);
    } else if (gamepad1.left_bumper) {
      farPowerAuto();
    } else if (gamepad1.right_bumper) {
      maxPowerAuto();
    } else if (gamepad1.triangle) {
      ((DcMotorEx) FlyWheel).setVelocity(theVelocity);
      //((DcMotorEx) HWheel).setVelocity(theVelocity);
    } else if (gamepad1.square) {
      ((DcMotorEx) FlyWheel).setVelocity(maxVelocity);
      //((DcMotorEx) HWheel).setVelocity(maxVelocity);
    } else {
      ((DcMotorEx) FlyWheel).setVelocity(0);
      //((DcMotorEx) HWheel).setVelocity(0);
        Feeder.setPosition(0.2);
      // The check below is in place to prevent stuttering with the servo. It checks if the servo is under manual control!
      if (!gamepad1.dpad_right && !gamepad1.dpad_left) {
          Feeder.setPosition(0.0);
      }
    }
  }

  /**
   * Describe this function...
   */
  private void farPowerAuto() {
    ((DcMotorEx) FlyWheel).setVelocity(farVelocity);
    In.setPower(-.9);
    if (((DcMotorEx) FlyWheel).getVelocity() >= farVelocity - 90) {
        Feeder.setPosition(0.2);
    } else {
        Feeder.setPosition(0.0);
    }
  }

}
