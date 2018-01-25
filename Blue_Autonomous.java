package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@Autonomous(name="BlueAutonomous", group="Pushbot")

public class Blue_Autonomous extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    private ElapsedTime     tempTime = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;
    double tX;
    double tY;
    double tZ;
    double rX;
    double rY;
    double rZ;
    int vustate;
    float number;

    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AceHAhf/////AAAAGXcP1HmPtEkhmi5YyP3W2S1gVnAvF2sEhNkTVsdCy4iDjm5aVrODZUpSZDIQZXVqOIqmjWvWcv1+56gq4NJ8h9P0m/MlKuqKbjcQNbSrxfQoBCJbD1G9gmkKFeaeKCrV/8ZQsipnso84dJHek4OfzMdvtKUU/QDrk+YCE7SWGMtZr7kFAWYss3vTpGv0WynOurUd+rly24nTP4qERK311b9MkK+uliO/slCL/vg6vANVX/NGSlXLRe4/nK0HitcsLrLjvcuRQJGeaYnzFB/ykuSZw3hFbHaSP45KH/fLivm0fql8ENaPyCLiNSDiqlSH553rXNiRenz3R9t8TW5YJjjAThy1U0F7GHtkGXKN/pfL";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT; // Use FRONT Camera (Change to BACK if you want to use that one)
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES; // Display Axes

        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        waitForStart();

        relicTrackables.activate(); // Activate Vuforia



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
        sleep(1000);


        //
        //Vuforia state setter
        //


        {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) { // Test to see if image is visable
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose(); // Get Positional value to use later

                if (vuMark == RelicRecoveryVuMark.LEFT) { // Test to see if Image is the "LEFT" image and display value.
                    vustate = 1;
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    vustate = 2;
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    vustate = 3;
                } else
                    vustate = 0;
            }
        }


        //
        //Our code
        //

        {
            robot.arm.setPosition(0);
            robot.arm2.setPosition(1);
            Thread.sleep(200);
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


        sleep(250);


        {
            robot.rightClaw.setPosition(.20);  //command continues previous
            waitOneFullHardwareCycle();
        }


        sleep(250);


        {
            robot.rightClaw.setPosition(.12);   //command continues previous
            waitOneFullHardwareCycle();
        }


        sleep(1000);


        {
            robot.rightClaw.setPosition(0.02);  //command puts color sensor into place
            waitOneFullHardwareCycle();
        }


        sleep(250);


        try {
            colorRead();
        } catch (InterruptedException e) {
            e.printStackTrace();
            telemetry.addData("FR", robot.motorFR.getCurrentPosition());
            telemetry.addData("FL", robot.motorFL.getCurrentPosition());
            telemetry.addData("BR", robot.motorBR.getCurrentPosition());
            telemetry.addData("BL", robot.motorBL.getCurrentPosition());
            telemetry.update();
        }


        sleep(500);                            //This line of code tells the color sensor to read the color, knock one of the balls of and reset it's position... This method needs a waitOnefullhardwarecycle(); I think and a busywait


        {
            robot.rightClaw.setPosition(1);
            waitOneFullHardwareCycle();
        }


        sleep(250);                            //puts claw back up

        {
            rotateFalse(.2, .2, .2, .2, 500, 500, 500, 500, 20);
        }
        sleep(1000);
        {
            reverseMotors(.2, .2, .2, .2, 2500, 2500, 2500, 2500, 20);

        }                                        //Moves motor forward, all the methods need waitOnefullhardwarecycle(); the 9th variable is for the Threadsleep(Time) which might be replacable with a busy wait
        {
            rotateTrue(.2, .2, .2, .2, 500, 500, 500, 500, 20);
        }
        {
            reverseMotors(.2, .2, .2, .2, 1000, 1000, 1000, 1000, 20);
        }






        {
            if (vustate == 1){
                rotateFalse(.2,.2,.2,.2,2500,2500,2500,2500,20);
            }
            else if (vustate == 2){
                rotateFalse(.15,.15,.15,.15,2050,2050,2050,2050,20);
            }
            else if (vustate == 3){
                rotateFalse(.15,.15,.15,.15,1700,1700,1700,1700,20);
            }
            else{
                rotateFalse(.15,.15,.15,.15,2050,2050,2050,2050,20);
            }
        }











        //turns robot, this is where fixing that one wheel that's spazzing out is crucial, We want the boxes to go in at an angle
        {
            moveMotors(.15,.15,.15,.15,2500,2500,2500,2500,20);
        }                                       //moves the robot forward so that we have the block in the the cryptobox and we're in the triangle
        {
            robot.arm.setPosition(1);
            robot.arm2.setPosition(0);
        }
        {
            robot.motorArm.setPower(1);
            Thread.sleep(600);
        }
        {
            robot.motorArm.setPower(0);
            Thread.sleep(300);
        }//opens the grabby boy
        {
            reverseMotors(.25,.25,.25,.25,500,500,500,500,20);
        }                                         //moves the robot back so that we can prepare for ramming speed
        {
            moveMotors(.25,.25,.25,.25,1500,1500,1500,1500,20);
        }

        //rams the cryptobox so that it's more likely that we get it in
        {
            reverseMotors(.25, .25, .25, .25, 250, 250, 250, 250, 20);
        }                                        //backs away from the box so that we aren't touching

        {
            motorThrowing2(1,1200);
        }





    }           //End Bracket









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
            rotateFalse(.25,.25,.25,.25,500,500,500,500,250);
            {robot.rightClaw.setPosition(1);
            }
            sleep(1000);
            //rotates the robot counterclockwise
            rotateTrue(.25,.25,.25,.25,500,500,500,500,250);
        }
        else if (robot.colorSensor.blue() >=2){


            rotateTrue(.25,.25,.25,.25,500,500,500,500,250);
            {robot.rightClaw.setPosition(1);
            }                   //moves the robot counterclockwise
            sleep(1000);
            rotateFalse(.25,.25,.25,.25,500,500,500,500,250);
        }                               //moves the robot clockwise
        else{
            moveMotors(0,0,0,0,0,0,0,0,1000);
            robot.rightClaw.setPosition(1);
        }






    }






    void moveMotors (double powerBL, double powerBR, double powerFL, double powerFR,
                     int posBL, int posBR, int posFL, int posFR, int time) throws InterruptedException
    {
        robot.motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.motorFL.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.motorBL.setTargetPosition(posBL);
        robot.motorBR.setTargetPosition(posBR);
        robot.motorFR.setTargetPosition(posFR);
        robot.motorFL.setTargetPosition(posFL);

        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        robot.motorBL.setPower(powerBL);
        robot.motorBR.setPower(powerBR);
        robot.motorFR.setPower(powerFR);
        robot.motorFL.setPower(powerFL);


        // BUSYWAIT... Don't show this to our CS professors :S

        while (opModeIsActive() &&
                (runtime.seconds() < time) &&
                (robot.motorFR.isBusy() && robot.motorFL.isBusy() && robot.motorBR.isBusy() && robot.motorBL.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", posBR, posBL, posFR, posFL);
            telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                    robot.motorFR.getCurrentPosition(),
                    robot.motorFL.getCurrentPosition(),
                    robot.motorBR.getCurrentPosition(),
                    robot.motorBL.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        robot.motorFR.setPower(0);
        robot.motorFL.setPower(0);
        robot.motorBR.setPower(0);
        robot.motorBL.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //  sleep(250);   // optional pause after each move
    }









    void reverseMotors (double powerBL, double powerBR, double powerFL, double powerFR,
                        int posBL, int posBR, int posFL, int posFR, int time) throws InterruptedException
    {
        robot.motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.motorFR.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.motorBL.setTargetPosition(posBL);
        robot.motorBR.setTargetPosition(posBR);
        robot.motorFR.setTargetPosition(posFR);
        robot.motorFL.setTargetPosition(posFL);

        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        robot.motorBL.setPower(powerBL);
        robot.motorBR.setPower(powerBR);
        robot.motorFR.setPower(powerFR);
        robot.motorFL.setPower(powerFL);


        // BUSYWAIT... Don't show this to our CS professors :S

        while (opModeIsActive() &&
                (runtime.seconds() < time) &&
                (robot.motorFR.isBusy() && robot.motorFL.isBusy() && robot.motorBR.isBusy() && robot.motorBL.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", posBR, posBL, posFR, posFL);
            telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                    robot.motorFR.getCurrentPosition(),
                    robot.motorFL.getCurrentPosition(),
                    robot.motorBR.getCurrentPosition(),
                    robot.motorBL.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        robot.motorFR.setPower(0);
        robot.motorFL.setPower(0);
        robot.motorBR.setPower(0);
        robot.motorBL.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //  sleep(250);   // optional pause after each move
    }








    void rotateFalse (double powerBL, double powerBR, double powerFL, double powerFR,
                      int posBL, int posBR, int posFL, int posFR, int time) throws InterruptedException     //counterclockwise
    {
        if (opModeIsActive()) {

            robot.motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.motorFL.setDirection(DcMotorSimple.Direction.FORWARD);

            robot.motorBL.setTargetPosition(posBL);
            robot.motorBR.setTargetPosition(posBR);
            robot.motorFR.setTargetPosition(posFR);
            robot.motorFL.setTargetPosition(posFL);

            robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.motorBL.setPower(powerBL);
            robot.motorBR.setPower(powerBR);
            robot.motorFR.setPower(powerFR);
            robot.motorFL.setPower(powerFL);


            // BUSYWAIT... Don't show this to our CS professors :S

            while (opModeIsActive() &&
                    (runtime.seconds() < time) &&
                    (robot.motorFR.isBusy() && robot.motorFL.isBusy() && robot.motorBR.isBusy() && robot.motorBL.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", posBR, posBL, posFR, posFL);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        robot.motorFR.getCurrentPosition(),
                        robot.motorFL.getCurrentPosition(),
                        robot.motorBR.getCurrentPosition(),
                        robot.motorBL.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.motorFR.setPower(0);
            robot.motorFL.setPower(0);
            robot.motorBR.setPower(0);
            robot.motorBL.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //  sleep(250);   // optional pause after each move

        }
    }






    void rotateTrue (double powerBL, double powerBR, double powerFL, double powerFR,
                     int posBL, int posBR, int posFL, int posFR, int time) throws InterruptedException   //clockwise
    {
        robot.motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.motorFL.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.motorBL.setTargetPosition(posBL);
        robot.motorBR.setTargetPosition(posBR);
        robot.motorFR.setTargetPosition(posFR);
        robot.motorFL.setTargetPosition(posFL);

        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        robot.motorBL.setPower(powerBL);
        robot.motorBR.setPower(powerBR);
        robot.motorFR.setPower(powerFR);
        robot.motorFL.setPower(powerFL);


        // BUSYWAIT... Don't show this to our CS professors :S

        while (opModeIsActive() &&
                (runtime.seconds() < time) &&
                (robot.motorFR.isBusy() && robot.motorFL.isBusy() && robot.motorBR.isBusy() && robot.motorBL.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d :%7d :7d :7d", posBR,  posBL, posFR, posFL);
            telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                    robot.motorFR.getCurrentPosition(),
                    robot.motorFL.getCurrentPosition(),
                    robot.motorBR.getCurrentPosition(),
                    robot.motorBL.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        robot.motorFR.setPower(0);
        robot.motorFL.setPower(0);
        robot.motorBR.setPower(0);
        robot.motorBL.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //  sleep(250);   // optional pause after each move
    }


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





    void rightMotors(double powerBL, double powerBR, double powerFL, double powerFR,
                     int posBL, int posBR, int posFL, int posFR, int time) throws InterruptedException{
        robot.motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.motorFR.setDirection(DcMotorSimple.Direction.REVERSE);


        robot.motorBL.setTargetPosition(posBL);
        robot.motorBR.setTargetPosition(posBR);
        robot.motorFR.setTargetPosition(posFR);
        robot.motorFL.setTargetPosition(posFL);

        robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        robot.motorBL.setPower(powerBL);
        robot.motorBR.setPower(powerBR);
        robot.motorFR.setPower(powerFR);
        robot.motorFL.setPower(powerFL);


        // BUSYWAIT... Don't show this to our CS professors :S

        while (opModeIsActive() &&
                (runtime.seconds() < time) &&
                (robot.motorFR.isBusy() && robot.motorFL.isBusy() && robot.motorBR.isBusy() && robot.motorBL.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Path1",  "Running to %7d :%7d :7d :7d", posBR,  posBL, posFR, posFL);
            telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                    robot.motorFR.getCurrentPosition(),
                    robot.motorFL.getCurrentPosition(),
                    robot.motorBR.getCurrentPosition(),
                    robot.motorBL.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        robot.motorFR.setPower(0);
        robot.motorFL.setPower(0);
        robot.motorBR.setPower(0);
        robot.motorBL.setPower(0);

        // Turn off RUN_TO_POSITION
        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //  sleep(250);   // optional pause after each move
    }

    void motorThrowing2 (double power, long time) throws InterruptedException {
        {
        robot.motorArm.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.motorArm.setPower(power);
        runtime.reset();
        Thread.sleep(time);
    }
        robot.motorFR.setPower(0);
        Thread.sleep(100);
    }


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
