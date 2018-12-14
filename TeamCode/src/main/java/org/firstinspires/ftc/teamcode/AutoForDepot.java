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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="AutoForDepot", group="OurRobot")
public class AutoForDepot extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double STRAIGHT_OUT = 0.25;
    public double LIFT_SPEED = 1;
    public double newColorPosition = .7;


    //strafeLeft Function
    int     newRightFrontTarget;
    int     moveCounts;
    int     newRightBackTarget;
    int     newLeftFrontTarget;
    int     newLeftBackTarget;



    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap, telemetry);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.liftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftBack.getCurrentPosition(),
                robot.rightBack.getCurrentPosition(),
                robot.leftFront.getCurrentPosition(),
                robot.rightFront.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        robot.liftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftArm.setTargetPosition(-25000);
        robot.liftArm.setPower(1);
        while (robot.liftArm.isBusy() && opModeIsActive()) {
            idle();
            telemetry.addLine()
                    .addData("lift arm", robot.liftArm.getCurrentPosition());
            telemetry.update();
        }
        robot.liftArm.setPower(0);

        //strafeLeft Function
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set Target and Turn On RUN_TO_POSITION
        robot.rightFront.setTargetPosition(-800);
        robot.rightBack.setTargetPosition(800);
        robot.leftFront.setTargetPosition(800);
        robot.leftBack.setTargetPosition(-800);

        //Do strafing left stuff
        robot.rightBack.setPower(.75);
        robot.rightFront.setPower(.75);
        robot.leftFront.setPower(1);
        robot.leftBack.setPower(1);

        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "Unhook");
            ;
            idle();
        }
        sleep(250);
        robot.allMotorsStop();

        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set Target and Turn On RUN_TO_POSITION
        robot.leftBack.setTargetPosition(2660);
        robot.leftFront.setTargetPosition(2660);
        robot.rightFront.setTargetPosition(2660);
        robot.rightBack.setTargetPosition(2660);

        //Do strafing right stuff
        robot.leftFront.setPower(1);
        robot.leftBack.setPower(1);
        robot.rightBack.setPower(1);
        robot.rightFront.setPower(1);

        while (robot.checkMotorIsBusy() && opModeIsActive()) {
            telemetry.addLine()
                    .addData("Task", "drive to minerals");
            telemetry.update();
            idle();
        }
        sleep(250);

        robot.allMotorsStop();

        robot.sampleArm.setPosition(.8);

        sleep(250);

        boolean foundMineral = false;

        while (robot.colorSensor.alpha() <= 22 && opModeIsActive()) {
            newColorPosition = robot.sampleArm.getPosition() - 0.01;
            robot.sampleArm.setPosition(newColorPosition);
            sleep(250);
            idle();
            if (robot.colorSensor.alpha() > 22) {
                robot.sampleArm.setPosition(newColorPosition - 0.05);
                sleep(100);
                foundMineral = true;
                continue;
            }
        }


        if (foundMineral == true) {
            if (robot.colorSensor.alpha() < 35) {

                robot.sampleArm.setPosition(1);
                sleep(250);

                robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                robot.leftBack.setTargetPosition(3700);
                robot.leftFront.setTargetPosition(3700);
                robot.rightFront.setTargetPosition(3700);
                robot.rightBack.setTargetPosition(3700);

                robot.leftFront.setPower(1);
                robot.leftBack.setPower(1);
                robot.rightBack.setPower(1);
                robot.rightFront.setPower(1);

                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "knock off gold + drive to crater");
                    telemetry.update();
                    idle();
                }
                sleep(250);

                robot.markerServo.setPosition(0);
                sleep(100);
            } else {

                robot.sampleArm.setPosition(1);
                sleep(250);

                robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Set Target and Turn On RUN_TO_POSITION
                robot.leftFront.setTargetPosition(2000);
                robot.leftBack.setTargetPosition(-2000);
                robot.rightFront.setTargetPosition(-2000);
                robot.rightBack.setTargetPosition(2000);

                //Do strafing right stuff
                robot.leftFront.setPower(1);
                robot.leftBack.setPower(1);
                robot.rightBack.setPower(1);
                robot.rightFront.setPower(1);

                while (robot.checkMotorIsBusy() && opModeIsActive()) {
                    telemetry.addLine()
                            .addData("Task", "strafe to other mineral");
                    telemetry.update();
                    idle();
                }
                sleep(250);

                robot.allMotorsStop();

                robot.sampleArm.setPosition(.7);
                sleep(250);

                boolean foundMineral2 = false;
                while (robot.colorSensor.alpha() <= 22 && opModeIsActive()) {
                    newColorPosition = robot.sampleArm.getPosition() - 0.01;
                    robot.sampleArm.setPosition(newColorPosition);
                    sleep(250);
                    idle();
                    if (robot.colorSensor.alpha() > 22) {
                        robot.sampleArm.setPosition(newColorPosition - 0.05);
                        if(robot.colorSensor.alpha()>35) {
                            sleep(100);
                            foundMineral2 = true;
                            continue;
                        }
                    }
                }

                if (foundMineral2 == true) {
                    robot.sampleArm.setPosition(1);
                    sleep(500);

                    robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    robot.leftBack.setTargetPosition(3700);
                    robot.leftFront.setTargetPosition(3700);
                    robot.rightFront.setTargetPosition(3700);
                    robot.rightBack.setTargetPosition(3700);

                    robot.leftFront.setPower(1);
                    robot.leftBack.setPower(1);
                    robot.rightBack.setPower(1);
                    robot.rightFront.setPower(1);

                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "knock off gold + drive to crater");
                        telemetry.update();
                        idle();
                    }
                    sleep(250);

                    robot.markerServo.setPosition(0);
                    sleep(100);
                } else {
                    // strafe to 3rd and final mineral
                    robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    // Set Target and Turn On RUN_TO_POSITION
                    robot.rightFront.setTargetPosition(5050);
                    robot.rightBack.setTargetPosition(-5050);
                    robot.leftFront.setTargetPosition(-5050);
                    robot.leftBack.setTargetPosition(5050);

                    //Do strafing left stuff
                    robot.rightBack.setPower(1);
                    robot.rightFront.setPower(1);
                    robot.leftFront.setPower(1);
                    robot.leftBack.setPower(1);

                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "driving to last mineral");
                        ;
                        idle();
                    }
                    sleep(250);
                    robot.allMotorsStop();

                    robot.sampleArm.setPosition(1);
                    sleep(250);

                    robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    robot.leftBack.setTargetPosition(3700);
                    robot.leftFront.setTargetPosition(3700);
                    robot.rightFront.setTargetPosition(3700);
                    robot.rightBack.setTargetPosition(3700);

                    robot.leftFront.setPower(1);
                    robot.leftBack.setPower(1);
                    robot.rightBack.setPower(1);
                    robot.rightFront.setPower(1);

                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "knock off gold");
                        telemetry.update();
                        idle();
                    }
                    sleep(250);
                    robot.allMotorsStop();

                    robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    robot.leftBack.setTargetPosition(-700);
                    robot.leftFront.setTargetPosition(-700);
                    robot.rightFront.setTargetPosition(700);
                    robot.rightBack.setTargetPosition(700);

                    robot.leftFront.setPower(1);
                    robot.leftBack.setPower(1);
                    robot.rightBack.setPower(1);
                    robot.rightFront.setPower(1);

                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "Turning to depot");
                        telemetry.update();
                        idle();
                    }
                    sleep(250);
                    robot.allMotorsStop();

                    robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    robot.leftBack.setTargetPosition(1000);
                    robot.leftFront.setTargetPosition(1000);
                    robot.rightFront.setTargetPosition(1000);
                    robot.rightBack.setTargetPosition(1000);

                    robot.leftFront.setPower(1);
                    robot.leftBack.setPower(1);
                    robot.rightBack.setPower(1);
                    robot.rightFront.setPower(1);

                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "drive into depot");
                        telemetry.update();
                        idle();
                    }
                    sleep(250);
                    robot.allMotorsStop();


                    robot.markerServo.setPosition(0);
                    sleep(100);
                }

            }
        }

        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.allMotorsStop();

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}