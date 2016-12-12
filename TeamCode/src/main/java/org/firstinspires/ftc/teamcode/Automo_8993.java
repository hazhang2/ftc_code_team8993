/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autonomous Test", group="Test")
// @Disabled
public class Automo_8993 extends LinearOpMode {

    Hardware5Motors robot           = new Hardware5Motors();
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    private ElapsedTime runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 720 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public static final double    BALL_PUSHER_POSITION_HIGH       = 0.25 ;
    public static final double    BALL_PUSHER_POSITION_MIDDLE       = 0. ;
    public static final double    BALL_PUSHER_POSITION_LOW      = 0. ;
    public static final double    BALL_PUSHER_POSITION_INC  = 0.01;


    public static final double SHOOTER_ACC_INC=0.1;
    public static final int SHOOTER_IACC_TICK=100;
    public static final int SHOOTER_IACC_TICK_MAX=900;


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        telemetry.addData("Say", "Autonomous Test");
        telemetry.update();
        robot.ballPusherServo.setPosition(BALL_PUSHER_POSITION_MIDDLE);
        double currentPos = BALL_PUSHER_POSITION_MIDDLE;


        waitForStart();

        startShooter();
        robot.waitForTick(2000);

        encoderDrive(DRIVE_SPEED,  1.8,  1.8, 5.0);  // S1: Forward 1.8 Inches with 5 Sec timeout
        robot.waitForTick(1000);
//        if(currentPos < BALL_PUSHER_POSITION_HIGH) {
//            currentPos = currentPos + BALL_PUSHER_POSITION_INC;
//            robot.ballPusherServo.setPosition(currentPos);
//        }
        robot.ballPusherServo.setPosition(BALL_PUSHER_POSITION_HIGH);
        robot.waitForTick(1000);
        stopShooter();
        encoderDrive(DRIVE_SPEED, 7, 7, 4.0);
        encoderDrive(TURN_SPEED,   15, -15, 4.0);  // S2: Turn Left 12 Inches with 4 Sec timeout
        robot.waitForTick(2000);
        encoderDrive(DRIVE_SPEED, 43, 43, 4.0);  // S3: Reverse 20 Inches with 4 Sec timeout
    }

    /*
 *  Method to perfmorm a relative move, based on encoder counts.
 *  Encoders are not reset as the move is based on the current position.
 *  Move will stop if any of three conditions occur:
 *  1) Move gets to the desired position
 *  2) Move runs out of time
 *  3) Driver stops the opmode running.
 */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void startShooter() throws InterruptedException {

        int ispeed=0;
        double shooterPower;

        while(ispeed <= SHOOTER_IACC_TICK_MAX) {

            int dd = ispeed / SHOOTER_IACC_TICK;
            shooterPower = dd * SHOOTER_ACC_INC + SHOOTER_ACC_INC;
            telemetry.addData("Say", "shooterpower increase to " + Double.toString(shooterPower));
            telemetry.update();
            robot.leftShooter.setPower(shooterPower);
            robot.rightShooter.setPower(shooterPower);

            ispeed = ispeed + 4;
            robot.waitForTick(40);
        }
    }

    public void stopShooter()
    {
        robot.leftShooter.setPower(0);
        robot.rightShooter.setPower(0);
    }
}
