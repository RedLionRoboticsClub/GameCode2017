package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@Autonomous(name="VuforiaTesting_Autonomous", group="Pushbot")

public class VuforiaTesting_Autonomous extends LinearOpMode {

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

        moveMotors(.5, .5, .5, .5, 1000, 1000, 1000, 1000, 1000);
        if (vustate == 1) {
            rotateTrue(.25,.25,.25,.25,1000,1000,1000,1000,1000);
        }
        else if (vustate == 2) {
            moveMotors(.5,.5,.5,.5,1000,1000,1000,1000,1000);
        }
        else if (vustate == 3) {
            rotateFalse(.25,.25,.25,.25,1000,1000,1000,1000,1000);
        }
        else {
            reverseMotors(.25,.25,.25,.25,1000,1000,1000,1000,1000);
        }
    }


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
            rotateFalse(.25,.25,.25,.25,500,500,500,500,500);
            {robot.rightClaw.setPosition(1);
            }
            sleep(1000);
            //rotates the robot counterclockwise
            rotateTrue(.25,.25,.25,.25,500,500,500,500,500);
        }
        else if (robot.colorSensor.blue() >=2){


            rotateTrue(.25,.25,.25,.25,500,500,500,500,500);
            {robot.rightClaw.setPosition(1);
            }                   //moves the robot counterclockwise
            sleep(1000);
            rotateFalse(.25,.25,.25,.25,500,500,500,500,500);
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

        tempTime.reset();
        while(tempTime.milliseconds() <= time);

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

        tempTime.reset();
        while(tempTime.milliseconds() <= time);

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
