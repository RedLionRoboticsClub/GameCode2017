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


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="AnthonyWillBeMad", group="Test")

public class AnthonyWillBeMad extends LinearOpMode {

    /* Declare OpMode members. */
   HardwareK9bot2 robot           = new HardwareK9bot2();              // Use a K9'shardware
    double          armPosition     = robot.ARM_HOME;                   // Servo safe position
 //   double          clawPosition    = robot.CLAW_HOME;                  // Servo safe position
  //  final double    CLAW_SPEED      = 0.01 ;                            // sets rate to move servo
    final double    ARM_SPEED       = 0.01 ;                            // sets rate to move servo

    @Override
    public void runOpMode() {
        double left;
        double right;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            if (gamepad1.right_trigger > 0) {

                robot.motorFR.setPower(-0.5);
                robot.motorFL.setPower(0.5);
                robot.motorBL.setPower(-0.5);
                robot.motorBR.setPower(0.5);
            }

            else if (gamepad1.left_trigger > 0) {
                robot.motorFR.setPower(0.5);
                robot.motorFL.setPower(-0.5);
                robot.motorBL.setPower(0.5);
                robot.motorBR.setPower(-0.5);
            }

            else {
                float yValue = gamepad1.left_stick_y;

                float xValue = gamepad1.right_stick_y;

                ///the usadjieiwebiqeuhowfeodfqwiwewenqwnrnower jkdfnasikjfmxbnc liawekjsmhdbf iukjqd


                xValue = Range.clip(xValue, -1, 1);
                yValue = Range.clip(yValue, -1, 1);

                if (gamepad1.left_bumper) {
                    robot.motorFL.setPower(-yValue/2);
                    robot.motorBL.setPower(-yValue/2);
                    robot.motorFR.setPower(-xValue/2);
                    robot.motorBR.setPower(-xValue/2);
                }

                else if (gamepad1.right_bumper) {
                    robot.motorFL.setPower(-yValue/4);
                    robot.motorBL.setPower(-yValue/4);
                    robot.motorFR.setPower(-xValue/4);
                    robot.motorBR.setPower(-xValue/4);
                }

                else {
                    robot.motorFL.setPower(-yValue);
                    robot.motorBL.setPower(-yValue);
                    robot.motorFR.setPower(-xValue);
                    robot.motorBR.setPower(-xValue);
                }

                }
            if (gamepad2.left_bumper) {
                if (gamepad2.a) {
                    robot.motorArm.setPower(-0.5);
                } else if (gamepad2.b) {
                    robot.motorArm.setPower(0.5);
                } else {
                    robot.motorArm.setPower(0);
                }
            }

            else if (gamepad2.right_bumper) {
                if (gamepad2.a) {
                    robot.motorArm.setPower(-0.25);
                } else if (gamepad2.b) {
                    robot.motorArm.setPower(0.25);
                } else {
                    robot.motorArm.setPower(0);
                }
            }

            else {
                if (gamepad2.a) {
                    robot.motorArm.setPower(-1);
                } else if (gamepad2.b) {
                    robot.motorArm.setPower(1);
                } else {
                    robot.motorArm.setPower(0);
                }
            }




            // Use gamepad Y & A raise and lower the arm
            // if (gamepad1.right_bumper)
            // armPosition += ARM_SPEED;
            // else if (gamepad1.left_bumper)
            // armPosition -= ARM_SPEED;

            // Use gamepad X & Y to open and close the claw
            if (gamepad2.x) {
                armPosition += ARM_SPEED;
            }
            else if (gamepad2.y) {
                armPosition -= ARM_SPEED;
            }

            // Move both servos to new position.
            armPosition  = Range.clip(armPosition, robot.ARM_MIN_RANGE, robot.ARM_MAX_RANGE);
            robot.arm.setPosition(armPosition);
            // clawPosition = Range.clip(clawPosition, robot.CLAW_MIN_RANGE, robot.CLAW_MAX_RANGE);
            // robot.claw.setPosition(clawPosition);

            // Send telemetry message to signify robot running;
            telemetry.addData("arm",   "%.2f", armPosition);
            // telemetry.addData("claw",  "%.2f", clawPosition);
            telemetry.update();

            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);

        }

    }

}
