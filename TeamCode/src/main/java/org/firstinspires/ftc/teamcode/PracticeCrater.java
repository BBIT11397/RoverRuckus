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

@Autonomous(name="Practice Crater", group="bbit")
public class PracticeCrater extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

   public double newColorPosition = .25;

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
        robot.liftArm.setTargetPosition(19512);
        robot.liftArm.setPower(1);
        while (robot.liftArm.isBusy() && opModeIsActive()) {
            idle();
            telemetry.addLine()
                    .addData("lift arm", robot.liftArm.getCurrentPosition());
            telemetry.update();
        }
        robot.liftArm.setPower(0);

        setUpMotors();
        // Set Target and Turn On RUN_TO_POSITION
        robot.rightFront.setTargetPosition(600);
        robot.rightBack.setTargetPosition(-600);
        robot.leftFront.setTargetPosition(-600);
        robot.leftBack.setTargetPosition(600);

        //Do strafing left stuff
        robot.rightBack.setPower(1);
        robot.rightFront.setPower(1);
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
        setUpMotors();
        robot.leftBack.setTargetPosition(2700);
        robot.leftFront.setTargetPosition(2700);
        robot.rightFront.setTargetPosition(2700);
        robot.rightBack.setTargetPosition(2700);

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

        setUpMotors();
        robot.rightFront.setTargetPosition(-600);
        robot.rightBack.setTargetPosition(600);
        robot.leftFront.setTargetPosition(600);
        robot.leftBack.setTargetPosition(-600);

        //Do strafing left stuff
        robot.rightBack.setPower(1);
        robot.rightFront.setPower(1);
        robot.leftFront.setPower(1);
        robot.leftBack.setPower(1);

        while (robot.checkMotorIsBusy() && opModeIsActive()) {
             telemetry.addLine()
                 .addData("Task", "reline");
            idle();
        }
        sleep(250);
        robot.allMotorsStop();

        robot.sampleArm.setPosition(.25);

        sleep(250);

        boolean foundMineral = false;

        double floor = robot.colorSensor.alpha();
        double mineralvalue = floor + 6;
        double lastReading = 0;

        while (robot.colorSensor.alpha() <= mineralvalue && opModeIsActive()) {
            telemetry.addLine()
                    .addData("alpha", robot.colorSensor.alpha());
            telemetry.update();
            newColorPosition = robot.sampleArm.getPosition() + 0.01;
            robot.sampleArm.setPosition(newColorPosition);
            sleep(250);
            idle();
            lastReading = robot.colorSensor.alpha();
            if (lastReading >= floor + 6) {
                sleep(250);
                foundMineral = true;
                continue;
            }
        }

        double lowestWhiteValue = floor + 25;
        double currentReading;
        double highestReading = lastReading;

        if (foundMineral == true) {
            robot.sampleArm.setPosition(newColorPosition + 0.04);
            sleep(250);

            telemetry.addLine()
                    .addData("alpha", highestReading);
            telemetry.update();

            sleep(250);


            currentReading = robot.colorSensor.alpha();

            if (currentReading > lastReading){
                highestReading = currentReading;
            }

            telemetry.addLine()
                    .addData("alpha", highestReading);
            telemetry.update();

            if (highestReading < lowestWhiteValue) {
                //gold is found
                robot.sampleArm.setPosition(0);
                sleep(500);

                setUpMotors();
                robot.leftBack.setTargetPosition(2500);
                robot.leftFront.setTargetPosition(2500);
                robot.rightFront.setTargetPosition(2500);
                robot.rightBack.setTargetPosition(2500);

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
                sleep(150);
            } else {

                robot.sampleArm.setPosition(0);
                sleep(150);

                setUpMotors();
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
                sleep(150);

                robot.allMotorsStop();

                robot.sampleArm.setPosition(.25);
                sleep(250);

                boolean foundMineral2 = false;
                floor = robot.colorSensor.alpha();
                mineralvalue = floor + 6;
                while (robot.colorSensor.alpha() <= mineralvalue && opModeIsActive()) {
                    newColorPosition = robot.sampleArm.getPosition() + 0.01;
                    robot.sampleArm.setPosition(newColorPosition);
                    telemetry.addLine()
                            .addData("alpha", floor);
                    telemetry.update();
                    sleep(250);
                    idle();
                    lastReading = robot.colorSensor.alpha();
                    highestReading = lastReading;
                    if (lastReading >= floor + 6) {
                        telemetry.addLine()
                                .addData("alpha", robot.colorSensor.alpha());
                        telemetry.update();
                        robot.sampleArm.setPosition(newColorPosition + 0.06);
                        sleep(1000);

                        currentReading = robot.colorSensor.alpha();

                        if(currentReading > lastReading){
                            highestReading = currentReading;
                        }
                        telemetry.addLine()
                                .addData("alpha", highestReading);
                        telemetry.update();
                        sleep(1000);

                        if(highestReading < lowestWhiteValue) {
                            foundMineral2 = true;
                            continue;
                        }
                    }
                }

                if (foundMineral2 == true) {

                    robot.sampleArm.setPosition(0);
                    sleep(500);

                    setUpMotors();
                    robot.leftBack.setTargetPosition(500);
                    robot.leftFront.setTargetPosition(-500);
                    robot.rightFront.setTargetPosition(500);
                    robot.rightBack.setTargetPosition(-500);

                    robot.leftBack.setPower(1);
                    robot.leftFront.setPower(1);
                    robot.rightFront.setPower(1);
                    robot.rightBack.setPower(1);

                    while (robot.checkMotorIsBusy() && opModeIsActive()) {
                        telemetry.addLine()
                                .addData("Task", "knock off gold + drive to crater");
                        telemetry.update();
                        idle();
                    }
                    sleep(150);

                    setUpMotors();
                    robot.leftBack.setTargetPosition(2500);
                    robot.leftFront.setTargetPosition(2500);
                    robot.rightFront.setTargetPosition(2500);
                    robot.rightBack.setTargetPosition(2500);

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
                    sleep(150);
                } else {

                    robot.sampleArm.setPosition(0);
                    sleep(250);
                    // strafe to 3rd and final mineral
                    setUpMotors();
                    robot.rightFront.setTargetPosition(4000);
                    robot.rightBack.setTargetPosition(-4000);
                    robot.leftFront.setTargetPosition(-4000);
                    robot.leftBack.setTargetPosition(4000);

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
                    sleep(150);
                    robot.allMotorsStop();

                    robot.sampleArm.setPosition(0);
                    sleep(150);

                    setUpMotors();
                    robot.leftBack.setTargetPosition(2500);
                    robot.leftFront.setTargetPosition(2500);
                    robot.rightFront.setTargetPosition(2500);
                    robot.rightBack.setTargetPosition(2500);

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
                    sleep(150);
                    robot.allMotorsStop();
                }

            }
        }


        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.allMotorsStop();
    }

    void setUpMotors() {
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
    }

 }