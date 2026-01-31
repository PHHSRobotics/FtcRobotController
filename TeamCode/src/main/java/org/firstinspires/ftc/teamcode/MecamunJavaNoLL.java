package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@TeleOp(name = "MecanumJavaNoLL (Blocks to Java)")
public class MecamunJavaNoLL extends LinearOpMode {
/** this is fun*/
  private DcMotor In;
  private DcMotor FlyWheel;
  private DcMotor BL;
  private DcMotor BR;
  private DcMotor FL;
  private DcMotor FR;
  private Servo Feeder;
  int maxVelocity;
  int bankVelocity;
  int farVelocity;

  /**
   * Describe this function...
   */


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
      float H;
    float V;
    float Pivot;
    Feeder = hardwareMap.get(Servo.class, "Feeder");
    In = hardwareMap.get(DcMotor.class, "In");
    FlyWheel = hardwareMap.get(DcMotor.class, "FlyWheel");
    BL = hardwareMap.get(DcMotor.class, "BL");
    BR = hardwareMap.get(DcMotor.class, "BR");
    FL = hardwareMap.get(DcMotor.class, "FL");
    FR = hardwareMap.get(DcMotor.class, "FR");

    FlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    FlyWheel.setDirection(DcMotor.Direction.REVERSE);
    Feeder.setPosition(0.0);
    bankVelocity = 1300;
    farVelocity = 1500;
    maxVelocity = 1950;
    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        if (gamepad1.a) {
          requestOpModeStop();
        }
        manualCoreHexAndServoControl();
        setFlywheelVelocity();
        H = gamepad1.left_stick_x;
        V = gamepad1.left_stick_y;
        Pivot = gamepad1.right_stick_x;
        BL.setPower(1 * (Pivot - (H + V)));
        BR.setPower(1 * (Pivot - (H - V)));
        FL.setPower(1 * (Pivot + (H - V)));
        FR.setPower(1 * (Pivot + H + V));
        telemetry.addData("Flywheel Velocity", ((DcMotorEx) FlyWheel).getVelocity());
        telemetry.addData("Flywheel Power", FlyWheel.getPower());
        /**telemetry.addData("Distance", distance);*/
        telemetry.update();
      }
    }
  }

  /**
   * Describe this function...
   */
  private void maxPowerAuto() {
    ((DcMotorEx) FlyWheel).setVelocity(maxVelocity);
    In.setPower(-.9);
    if (((DcMotorEx) FlyWheel).getVelocity() >= maxVelocity - 100) {
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
    } else if (gamepad1.left_bumper) {
      farPowerAuto();
    } else if (gamepad1.right_bumper) {
      maxPowerAuto();
    } else if (gamepad1.y) {
      ((DcMotorEx) FlyWheel).setVelocity(bankVelocity);
    } else if (gamepad1.triangle) {
      bankShotAuto();
    } else if (gamepad1.square) {
      ((DcMotorEx) FlyWheel).setVelocity(maxVelocity);
    } else {
      ((DcMotorEx) FlyWheel).setVelocity(0);
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
    if (((DcMotorEx) FlyWheel).getVelocity() >= farVelocity - 100) {
        Feeder.setPosition(0.2);
    } else {
        Feeder.setPosition(0.0);
    }
  }

}
