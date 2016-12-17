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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Servo TeleOp", group="Test")
@Disabled
public class ServoTeleOp extends LinearOpMode {

    HardwareServo robot           = new HardwareServo();

    // double          clawOffset      = 0;
    public static final double    BALL_PUSHER_POSITION_HIGH       = 0.4 ;
    public static final double    BALL_PUSHER_POSITION_LOW      = 0. ;

    @Override
    public void runOpMode() throws InterruptedException {
        double leftMotorPower;
        double rightMotorPower;
        // double rightShooterPower;
        // double leftShooterPower;

        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello Person!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.a)
            {
                robot.rotatingServo.setPosition(BALL_PUSHER_POSITION_LOW);
            }
            if(gamepad1.b)
            {
                robot.rotatingServo.setPosition(BALL_PUSHER_POSITION_HIGH);
            }
//            rightMotorPower = 0;
//            leftMotorPower  = 0;
            // rightShooterPower = -gamepad1.right_stick_y;
            // leftShooterPower = -gamepad1.left_stick_y;

//                robot.leftMotor.setPower(leftMotorPower);
//                robot.rightMotor.setPower(rightMotorPower);
                // robot.rightShooter.setPower(rightShooterPower);
                // robot.leftShooter.setPower(leftShooterPower);

                // robot.leftShooter.getCurrentPosition();
                // robot.rightShooter.getCurrentPosition();


            robot.waitForTick(40);
        }
    }
}
