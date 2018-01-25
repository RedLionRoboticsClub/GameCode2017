/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="BlueLeftAutonomous", group="GameCode")

public class BlueLeft_Autonomous extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds

        {
            robot.arm.setPosition(0);
            robot.arm2.setPosition(1);
        }
        {
            robot.motorArm.setPower(1);
            Thread.sleep(800);
        }
        {
            robot.motorArm.setPower(0);
            Thread.sleep(800);
        }
        {
            robot.rightClaw.setPosition(.3);  //command to move the color sensor into position slowly
            waitOneFullHardwareCycle();
        }
        sleep(1000);
        {
            robot.rightClaw.setPosition(.20);  //command continues previous
            waitOneFullHardwareCycle();
        }
        sleep(1000);
        {
            robot.rightClaw.setPosition(.1);   //command continues previous
            waitOneFullHardwareCycle();
        }
        sleep(1000);
        {
            robot.rightClaw.setPosition(0.02);  //command puts color sensor into place
            waitOneFullHardwareCycle();
        }
        sleep(1000);
        try {
            colorRead();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        //This line of code tells the color sensor to read the color, knock one of the balls of and reset it's position... This method needs a waitOnefullhardwarecycle(); I think and a busywait

        {robot.rightClaw.setPosition(1);
            waitOneFullHardwareCycle();
        }
        sleep(1000);                            //puts claw back up


        {
            moveMotors(.25,.25,.25,.25,2000,2000,2000,2000,1000);

        }                                        //Moves motor forward, all the methods need waitOnefullhardwarecycle(); the 9th variable is for the Threadsleep(Time) which might be replacable with a busy wait
        {
            moveMotors(0,.25,0,.25,0,4500,0,4500,3000);
        }                                           //turns robot, this is where fixing that one wheel that's spazzing out is crucial, We want the boxes to go in at an angle
        {
            moveMotors(.25,.25,.25,.25,1337,1337,1337,1337,1000);
        }                                       //moves the robot forward so that we have the block in the the cryptobox and we're in the triangle
        {
            robot.arm.setPosition(1);
            robot.arm2.setPosition(0);
        }
        {
            robot.motorArm.setPower(1);
            Thread.sleep(1200);
        }
        {
            robot.motorArm.setPower(0);
            Thread.sleep(1200);
        }
        sleep(2000);//opens the grabby boy
        {
            reverseMotors(.25,.25,.25,.25,500,500,500,500,1000);
        }                                         //moves the robot back so that we can prepare for ramming speed
        {
            moveMotors(.25,.25,.25,.25,1500,1500,1500,1500,1000);
        }                                          //rams the cryptobox so that it's more likely that we get it in
        reverseMotors(.25,.25,.25,.25,250,250,250,250,1000);
    }                                               //backs away from the box so that we aren't touching


    void motorThrowing (double power, long time) throws InterruptedException
    {
        robot.motorFR.setPower(power);
        runtime.reset();
        sleep(time);

        robot.motorFR.setPower(0);
        sleep(100);
    }
    public void colorRead() throws InterruptedException
    {
        robot.colorSensor.enableLed(true);
        sleep(2000);
        float hsvValues[] = {0F, 0F, 0F};
        Color.RGBToHSV(robot.colorSensor.red() * 8, robot.colorSensor.green() * 8, robot.colorSensor.blue() * 8, hsvValues);
        //This gives the robot time to turn on the led and read the colorSensor stuff


        if (robot.colorSensor.red() >= 2) {


            //rotates the robot clockwise
            rotateTrue(.25,.25,.25,.25,500,500,500,500,500);
            {robot.rightClaw.setPosition(1);
            }

            //rotates the robot counterclockwise
            rotateFalse(.25,.25,.25,.25,500,500,500,500,500);
        }
        else {


            rotateFalse(.25,.25,.25,.25,500,500,500,500,500);
            {robot.rightClaw.setPosition(1);
            }                   //moves the robot counterclockwise

            rotateTrue(.25,.25,.25,.25,500,500,500,500,500);
        }                               //moves the robot clockwise
    }
    void moveMotors (double powerBL, double powerBR, double powerFL, double powerFR,
                     int posBL, int posBR, int posFL, int posFR, int time) throws InterruptedException
    {
        robot.motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.motorFL.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.motorBL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.motorBR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.motorFR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.motorFL.setMode(DcMotor.RunMode.RESET_ENCODERS);

        robot.motorBL.setTargetPosition(posBL);
        robot.motorBR.setTargetPosition(posBR);
        robot.motorFR.setTargetPosition(posFR);
        robot.motorFL.setTargetPosition(posFL);

        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robot.motorBL.setPower(powerBL);
        robot.motorBR.setPower(powerBR);
        robot.motorFR.setPower(powerFR);
        robot.motorFL.setPower(powerFL);
        // BUSYWAIT... Don't show this to our CS professors :S

        Thread.sleep(time);

        /**      motorBR.setPower(0.0);
         motorBL.setPower(0.0);
         motorFL.setPower(0);
         motorFR.setPower(0);
         motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);             **/
        // target has reached destination
//        motorLeft.setPower(0.0);
//        motorRight.setPower(0.0);
//        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    void reverseMotors (double powerBL, double powerBR, double powerFL, double powerFR,
                        int posBL, int posBR, int posFL, int posFR, int time) throws InterruptedException
    {
        robot.motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.motorFR.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.motorBL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.motorBR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.motorFR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.motorFL.setMode(DcMotor.RunMode.RESET_ENCODERS);

        robot.motorBL.setTargetPosition(posBL);
        robot.motorBR.setTargetPosition(posBR);
        robot.motorFR.setTargetPosition(posFR);
        robot.motorFL.setTargetPosition(posFL);

        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robot.motorBL.setPower(powerBL);
        robot.motorBR.setPower(powerBR);
        robot.motorFR.setPower(powerFR);
        robot.motorFL.setPower(powerFL);
        // BUSYWAIT... Don't show this to our CS professors :S

        Thread.sleep(time);

        /**      motorBR.setPower(0.0);
         motorBL.setPower(0.0);
         motorFL.setPower(0);
         motorFR.setPower(0);
         motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);             **/
        // target has reached destination
//        motorLeft.setPower(0.0);
//        motorRight.setPower(0.0);
//        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitOneFullHardwareCycle();
    }
    void rotateFalse (double powerBL, double powerBR, double powerFL, double powerFR,
                      int posBL, int posBR, int posFL, int posFR, int time) throws InterruptedException     //counterclockwise
    {
        robot.motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.motorFL.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.motorBL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.motorBR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.motorFR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.motorFL.setMode(DcMotor.RunMode.RESET_ENCODERS);

        robot.motorBL.setTargetPosition(posBL);
        robot.motorBR.setTargetPosition(posBR);
        robot.motorFR.setTargetPosition(posFR);
        robot.motorFL.setTargetPosition(posFL);

        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robot.motorBL.setPower(powerBL);
        robot.motorBR.setPower(powerBR);
        robot.motorFR.setPower(powerFR);
        robot.motorFL.setPower(powerFL);
        // BUSYWAIT... Don't show this to our CS professors :S

        Thread.sleep(time);

        /**      motorBR.setPower(0.0);
         motorBL.setPower(0.0);
         motorFL.setPower(0);
         motorFR.setPower(0);
         motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);             **/
        // target has reached destination
//        motorLeft.setPower(0.0);
//        motorRight.setPower(0.0);
//        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitOneFullHardwareCycle();
    }
    void rotateTrue (double powerBL, double powerBR, double powerFL, double powerFR,
                     int posBL, int posBR, int posFL, int posFR, int time) throws InterruptedException   //clockwise
    {
        robot.motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.motorFL.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.motorBL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.motorBR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.motorFR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.motorFL.setMode(DcMotor.RunMode.RESET_ENCODERS);

        robot.motorBL.setTargetPosition(posBL);
        robot.motorBR.setTargetPosition(posBR);
        robot.motorFR.setTargetPosition(posFR);
        robot.motorFL.setTargetPosition(posFL);

        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robot.motorBL.setPower(powerBL);
        robot.motorBR.setPower(powerBR);
        robot.motorFR.setPower(powerFR);
        robot.motorFL.setPower(powerFL);
        // BUSYWAIT... Don't show this to our CS professors :S

        Thread.sleep(time);

        /**      motorBR.setPower(0.0);
         motorBL.setPower(0.0);
         motorFL.setPower(0);
         motorFR.setPower(0);
         motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);             **/
        // target has reached destination
//        motorLeft.setPower(0.0);
//        motorRight.setPower(0.0);
//        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitOneFullHardwareCycle();
    }
    void rightMotors(double powerBL, double powerBR, double powerFL, double powerFR,
                     int posBL, int posBR, int posFL, int posFR, int Time) throws InterruptedException{
        robot.motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.motorFR.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.motorBL.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.motorBR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.motorFR.setMode(DcMotor.RunMode.RESET_ENCODERS);
        robot.motorFL.setMode(DcMotor.RunMode.RESET_ENCODERS);

        robot.motorBL.setTargetPosition(posBL);
        robot.motorBR.setTargetPosition(posBR);
        robot.motorFR.setTargetPosition(posFR);
        robot.motorFL.setTargetPosition(posFL);

        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robot.motorBL.setPower(powerBL);
        robot.motorBR.setPower(powerBR);
        robot.motorFR.setPower(powerFR);
        robot.motorFL.setPower(powerFL);
        // BUSYWAIT... Don't show this to our CS professors :S

        Thread.sleep(Time);
        waitOneFullHardwareCycle();
    }
}
