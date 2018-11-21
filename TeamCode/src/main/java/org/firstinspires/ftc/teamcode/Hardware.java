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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class Hardware
{
    /* Public OpMode members. */
    public DcMotor  leftFront   = null;
    public DcMotor  leftBack  = null;
    public DcMotor  rightFront  = null;
    public DcMotor  rightBack  = null;
    public DcMotor  liftArm    = null;

    int     newRightFrontTarget;
    int     moveCounts;
    int     newRightBackTarget;
    int     newLeftFrontTarget;
    int     newLeftBackTarget;

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public double LIFT_SPEED = 1;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    private Telemetry telemetry;

    DigitalChannel magnetSwitch;  // Hardware Device Object

    /* Constructor */
    public Hardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry telemetry) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        telemetry.addData("hardware init:" , "enter");
        telemetry.update();

        // Define and Initialize Motors
        leftBack  = hwMap.get(DcMotor.class, "leftBack");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        leftFront  = hwMap.get(DcMotor.class, "leftFront");
        rightBack = hwMap.get(DcMotor.class, "rightBack");
        liftArm = hwMap.get(DcMotor.class, "liftArm");

        // get a reference to our digitalTouch object.
        magnetSwitch= hwMap.get(DigitalChannel.class, "magnetSwitch");

        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftBack.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set the digital channel to input.
        magnetSwitch.setMode(DigitalChannel.Mode.INPUT);

       /*
        liftArm.setPower(LIFT_SPEED);

        while (magnetSwitch.getState() == true) {
            liftArm.isBusy();
            telemetry.addData("hardware init:" , "Reseting liftArm");
            telemetry.update();
        }

        liftArm.setPower(0);

        liftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
*/
        telemetry.addData("hardware init:" , "exit");
        telemetry.update();
    }

    public void strafeLeft(double speed , int inches) {
        // Determine new target position, and pass to motor controller
        moveCounts = (int)(inches * COUNTS_PER_INCH);
        newRightFrontTarget = rightFront.getCurrentPosition() + moveCounts;
        newRightBackTarget = rightBack.getCurrentPosition() + moveCounts;
        newLeftFrontTarget = leftFront.getCurrentPosition() + moveCounts;
        newLeftBackTarget = leftBack.getCurrentPosition() + moveCounts;

        // Set Target and Turn On RUN_TO_POSITION
        rightFront.setTargetPosition(newRightFrontTarget);
        rightBack.setTargetPosition(newRightBackTarget);
        leftFront.setTargetPosition(newLeftFrontTarget);
        leftBack.setTargetPosition(newLeftBackTarget);

        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Do strafing left stuff
        rightBack.setPower(1);
        rightFront.setPower(-1);
        leftFront.setPower(1);
        leftBack.setPower(-1);
    }
    public void strafeRight(double speed , int inches) {
        // Determine new target position, and pass to motor controller
        moveCounts = (int)(inches * COUNTS_PER_INCH);
        newLeftFrontTarget = leftFront.getCurrentPosition() + moveCounts;
        newLeftBackTarget = leftBack.getCurrentPosition() + moveCounts;
        newRightFrontTarget = rightFront.getCurrentPosition() + moveCounts;
        newRightBackTarget = rightBack.getCurrentPosition() + moveCounts;

        // Set Target and Turn On RUN_TO_POSITION
        leftFront.setTargetPosition(newLeftFrontTarget);
        leftBack.setTargetPosition(newLeftBackTarget);
        rightFront.setTargetPosition(newRightFrontTarget);
        rightBack.setTargetPosition(newRightBackTarget);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Do strafing right stuff
        leftFront.setPower(-1);
        leftBack.setPower(1);
        rightBack.setPower(-1);
        rightFront.setPower(1);
    }


}

