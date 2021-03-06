package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Vector;

/**
 * Created by Lea & Kevin on 1/14/2017.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Chan_SensorColor", group ="Tests")
public class Chan_SensorColor extends LinearOpMode
{

  DcMotor motorLeft;
  DcMotor motorRight;
  DcMotor motorThrow;
  ColorSensor colorSensor;
  private ElapsedTime runtime = new ElapsedTime();
  //We use this to make a new variable called runtime we can use to run the throwing motor off of time.
  private static final int MOTOR_TICKS = 1120;
  // in cm
  private static final double WHEEL_CIRCUMFERENCE = 32.5;
  // this is calculated from center of motor wheels. Is the radius of its turn
  private static final double ROBOT_WIDTH = 34.8;
  private static final double ROBOT_LENGTH = 37.5;

  private Vector<String> log;

  private void addLog (String entry)
  {
    log.add(entry);

    for (int i = 0; i < log.size(); i++)
    {
      telemetry.addData(String.format("%d ",i), log.get(i));
    }
    telemetry.update();
  }

  @Override
  public void runOpMode() throws InterruptedException
  {
    log = new Vector<String>();
    addLog("Thread started");

    // Controller
    motorLeft  = this.hardwareMap.dcMotor.get("motorBL");
    motorRight = this.hardwareMap.dcMotor.get("motorBR");
    motorThrow = this.hardwareMap.dcMotor.get("motorFL");
    colorSensor = this.hardwareMap.colorSensor.get("color sensor");

    motorRight.setDirection(DcMotorSimple.Direction.REVERSE);

    motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    motorThrow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    waitForStart();




    colorRead();



    addLog("Thread done");
  }



  void motorThrowing (double power, long time) throws InterruptedException
  {
    motorThrow.setPower(power);
    runtime.reset();
    sleep(time);

    motorThrow.setPower(0);
    sleep(100);
  }

 /*   void stopThrowing (double powers, double times) throws InterruptedException
    {
        motorThrow.setPower(powers);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < times))
        {
            idle();
            //Stops Throwing probably a waste of time but is nessarry for now
        }}
   */

  //  void motorThrowing (double power) throws InterruptedException {

//        motorThrow.setPower(power);
  //      Thread.sleep(2);
  //}

  /*
  Assumes robot is parked in front of beacon, within 7cm
   */
  public void colorRead() throws InterruptedException
  {
    colorSensor.enableLed(false);
    sleep(1000);
    float hsvValues[] = {0F, 0F, 0F};
    Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
    // send the info back to driver station using telemetry function.
    addLog(String.format("Clear", colorSensor.alpha()));
    addLog(String.format("Red  ", colorSensor.red()));
    addLog(String.format("Green", colorSensor.green()));
    addLog(String.format("Blue ", colorSensor.blue()));
    addLog(String.format("Hue", hsvValues[0]));

    if (colorSensor.blue() >= 3)
    {
      //move right wheel forward
      addLog("Blue was spotted");
      motorThrowing(.5,5000);
    }
    else if (colorSensor.red() >= 3)
    {
      //move left wheel forward
      addLog("Red was spotted");
      motorThrowing(-.5,5000);
    }
  }
}
