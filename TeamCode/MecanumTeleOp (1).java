package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "CloseRed (Blocks to Java)")
public class CloseRed extends LinearOpMode {

  private DcMotor BLeft;
  private DcMotor BRight;
  private DcMotor FLeft;
  private DcMotor FRight;
  private DcMotor FlyWheel;
  private DcMotor Feeder;
  private CRServo servo;
  private DcMotor In;

  int bankVelocity;
  int farVelocity;
  int maxVelocity;

  /**
   * Describe this function...
   */
  private void SRight() {
    BLeft.setPower(0.6 * 1);
    BRight.setPower(0.6 * 1);
    FLeft.setPower(0.6 * -1);
    FRight.setPower(0.6 * -1);
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
    BLeft = hardwareMap.get(DcMotor.class, "BLeft");
    BRight = hardwareMap.get(DcMotor.class, "BRight");
    FLeft = hardwareMap.get(DcMotor.class, "FLeft");
    FRight = hardwareMap.get(DcMotor.class, "FRight");
    FlyWheel = hardwareMap.get(DcMotor.class, "FlyWheel");
    Feeder = hardwareMap.get(DcMotor.class, "Feeder");
    servo = hardwareMap.get(CRServo.class, "servo");
    In = hardwareMap.get(DcMotor.class, "In");

    FlyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    FlyWheel.setDirection(DcMotor.Direction.FORWARD);
    Feeder.setDirection(DcMotor.Direction.REVERSE);
    servo.setPower(0);
    bankVelocity = 1300;
    farVelocity = 1500;
    maxVelocity = 1950;
    waitForStart();
    if (opModeIsActive()) {
      DriveB();
      sleep(1200);
      StopDrive();
      In.setPower(-0.7);
      bankShotAuto();
      sleep(10000);
      StopLancher();
      TurnR();
      sleep(600);
      StopDrive();
      DriveF();
      sleep(500);
      StopDrive();
      DriveB();
      sleep(500);
      StopDrive();
      TurnL();
      sleep(600);
      StopDrive();
      bankShotAuto();
      sleep(10000);
      StopLancher();
      SRight();
      sleep(500);
      StopDrive();
      requestOpModeStop();
    }
  }

  /**
   * Describe this function...
   */
  private void DriveF() {
    BLeft.setPower(0.6 * -1);
    BRight.setPower(0.6 * 1);
    FLeft.setPower(0.6 * -1);
    FRight.setPower(0.6 * 1);
  }

  /**
   * Describe this function...
   */
  private void bankShotAuto() {
    ((DcMotorEx) FlyWheel).setVelocity(bankVelocity);
    servo.setPower(1);
    if (((DcMotorEx) FlyWheel).getVelocity() >= bankVelocity - 50) {
      Feeder.setPower(1);
    } else {
      Feeder.setPower(0);
    }
  }

  /**
   * Describe this function...
   */
  private void SLeft() {
    BLeft.setPower(0.6 * -1);
    BRight.setPower(0.6 * -1);
    FLeft.setPower(0.6 * 1);
    FRight.setPower(0.6 * 1);
  }

  /**
   * Describe this function...
   */
  private void DriveB() {
    BLeft.setPower(0.6 * 1);
    BRight.setPower(0.6 * -1);
    FLeft.setPower(0.6 * 1);
    FRight.setPower(0.6 * -1);
  }

  /**
   * Describe this function...
   */
  private void farPowerAuto() {
    ((DcMotorEx) FlyWheel).setVelocity(farVelocity);
    servo.setPower(1);
    if (((DcMotorEx) FlyWheel).getVelocity() >= farVelocity - 100) {
      Feeder.setPower(1);
    } else {
      Feeder.setPower(0);
    }
  }

  /**
   * Describe this function...
   */
  private void TurnR() {
    BLeft.setPower(0.6 * -1);
    BRight.setPower(0.6 * -1);
    FLeft.setPower(0.6 * -1);
    FRight.setPower(0.6 * -1);
  }

  /**
   * Describe this function...
   */
  private void TurnL() {
    BLeft.setPower(0.6 * 1);
    BRight.setPower(0.6 * 1);
    FLeft.setPower(0.6 * 1);
    FRight.setPower(0.6 * 1);
  }

  /**
   * Describe this function...
   */
  private void maxPowerAuto() {
    ((DcMotorEx) FlyWheel).setVelocity(maxVelocity);
    servo.setPower(1);
    if (((DcMotorEx) FlyWheel).getVelocity() >= maxVelocity - 150) {
      Feeder.setPower(1);
    } else {
      Feeder.setPower(0);
    }
  }

  /**
   * Describe this function...
   */
  private void StopDrive() {
    BLeft.setPower(0);
    BRight.setPower(0);
    FLeft.setPower(0);
    FRight.setPower(0);
  }

  /**
   * Describe this function...
   */
  private void StopLancher() {
    FlyWheel.setPower(0);
    Feeder.setPower(0);
    servo.setPower(0);
  }
}
