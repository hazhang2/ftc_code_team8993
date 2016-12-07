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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp 2", group="Test")
// @Disabled
public class TeleOp extends LinearOpMode {

    HardwareTest robot           = new HardwareTest();

    // double          clawOffset      = 0;
    // public static final double    ARM_EXTENDED_POSITION       = 0.08 ;
    // public static final double    ARM_RETRACTED_POSITION      = 0.02 ;
    public static final double MOTOR_ACC_INC=0.05;
    public static final double SPEED_FACTOR=1.5;

    @Override
    public void runOpMode() throws InterruptedException {
        double leftMotorPower=0.0;
        double rightMotorPower=0.0;
        double leftMoterPowerb4=0.0;
        double rightMoterPowerb4=0.0;

        robot.init(hardwareMap);

        telemetry.addData("Say", "Tele Op");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

           // if(gamepad1.a)
           // {
           //     robot.rotatingServo.setPosition(ARM_EXTENDED_POSITION);
           // }

            if(Math.abs(gamepad1.right_stick_y-rightMoterPowerb4*SPEED_FACTOR) > MOTOR_ACC_INC)
            {
                if(gamepad1.right_stick_y > rightMoterPowerb4*SPEED_FACTOR) {
                    rightMotorPower = rightMoterPowerb4 + MOTOR_ACC_INC;
                }
                else{
                    rightMotorPower = rightMoterPowerb4 - MOTOR_ACC_INC;
                }
                rightMoterPowerb4=rightMotorPower;
            }

            if(Math.abs(gamepad1.left_stick_y-leftMoterPowerb4*SPEED_FACTOR) > MOTOR_ACC_INC)
            {
                if(gamepad1.left_stick_y > leftMoterPowerb4*SPEED_FACTOR) {
                    leftMotorPower = leftMoterPowerb4 + MOTOR_ACC_INC;
                }
                else{
                    leftMotorPower = leftMoterPowerb4 - MOTOR_ACC_INC;
                }
                leftMoterPowerb4=leftMotorPower;
            }

            // rightShooterPower = -gamepad1.right_stick_y;
            // leftShooterPower = -gamepad1.left_stick_y;

            robot.leftMotor.setPower(-leftMotorPower);
            robot.rightMotor.setPower(-rightMotorPower);
            telemetry.addData("Say", "Moter power (left,right),"+Double.toString(leftMotorPower)+
                    ","+Double.toString(rightMotorPower));
            telemetry.update();
//            if(gamepad1.a)
//            {
//                robot.rightShooter.setPower(1);
//                robot.leftShooter.setPower(1);
//            }
//            if(gamepad1.b)
//            {
//                robot.rightShooter.setPower(0);
//                robot.leftShooter.setPower(0);
//            }
                // robot.leftShooter.getCurrentPosition();
                // robot.rightShooter.getCurrentPosition();


            robot.waitForTick(40);
        }
    }
}
