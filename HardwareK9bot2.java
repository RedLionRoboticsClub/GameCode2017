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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Servo channel:  Servo to raise/lower arm: "arm"
 * Servo channel:  Servo to open/close claw: "claw"
 *
 * Note: the configuration of the servos is such that:
 *   As the arm servo approaches 0, the arm position moves up (away from the floor).
 *   As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class HardwareK9bot2
{
    /* Public OpMode members. */
    public DcMotor  motorFL   = null;
    public DcMotor  motorFR   = null;
    public DcMotor  motorBL   = null;
    public DcMotor  motorBR   = null;
    public DcMotor  motorArm  = null;
    public DcMotor  motorFan  = null;
    public DcMotor  relic     = null;
    public Servo    arm       = null;
    public Servo    arm2      = null;
    public Servo    armJ      = null;
    //public Servo    claw        = null;

    public final static double ARM_HOME = 1;
    public final static double ARM_HOME2 = 0;
    public final static double ARM_HOMEJ = 1;
    //public final static double CLAW_HOME = 0.2;
    public final static double ARM_MIN_RANGE  = 0;
    public final static double ARM_MIN_RANGE2  = 0;
    public final static double ARM_MAX_RANGE  = 1;
    public final static double ARM_MAX_RANGE2  = 1;
    public final static double ARM_MIN_RANGEJ  = 0;
    public final static double ARM_MAX_RANGEJ  = 0.90;
    //public final static double CLAW_MIN_RANGE  = 0.20;
    //public final static double CLAW_MAX_RANGE  = 0.7;

    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareK9bot2() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        motorFL  = hwMap.get(DcMotor.class, "motorFL");
        motorFR = hwMap.get(DcMotor.class, "motorFR");
        motorBL = hwMap.get(DcMotor.class, "motorBL");
        motorBR = hwMap.get(DcMotor.class, "motorBR");
        motorArm = hwMap.get(DcMotor.class, "motorArm");
        motorFan = hwMap.get(DcMotor.class, "motorFan");
        relic = hwMap.get(DcMotor.class, "relic");
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorArm.setPower(0);
        motorFan.setPower(0);
        relic.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFan.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        relic.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        arm  = hwMap.get(Servo.class, "crservo");
        arm2 = hwMap.get(Servo.class, "crservo2");
        armJ = hwMap.get(Servo.class, "crservoJ");
        //      claw = hwMap.get(Servo.class, "claw");
        arm.setPosition(ARM_HOME);
        arm2.setPosition(ARM_HOME2);
        armJ.setPosition(ARM_HOMEJ);
//        claw.setPosition(CLAW_HOME);
    }
}

