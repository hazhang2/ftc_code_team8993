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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Motor Test", group="Test")
// @Disabled
public class Motor_test extends LinearOpMode {

    HardwareTest robot           = new HardwareTest();

    // double          clawOffset      = 0;
    // public static final double    ARM_EXTENDED_POSITION       = 0.08 ;
    // public static final double    ARM_RETRACTED_POSITION      = 0.02 ;

    @Override
    public void runOpMode() throws InterruptedException {
        double leftMotorPower;
        double rightMotorPower;
        // double rightShooterPower;
        // double leftShooterPower;
        // double rotatingServo;

        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello Person!");
        telemetry.update();

        waitForStart();
        int iaccelarate=0;
        double shooterpower=0;

        while (opModeIsActive()) {

//            robot.rightShooter.setPower(0.1);

            if(iaccelarate%100==0)
            {
                int dd=iaccelarate/100;
                shooterpower=dd*0.1+0.1;
                telemetry.addData("Say","shooterpower increase to "+Double.toString(shooterpower));
                telemetry.update();
                robot.leftShooter.setPower(shooterpower);
                robot.rightShooter.setPower(shooterpower);
            }
            if(iaccelarate<900)
            {
                iaccelarate++;
            }

//            rightMotorPower = -gamepad1.right_stick_y;
//            leftMotorPower  = -gamepad1.left_stick_y;
//
//                robot.leftMotor.setPower(leftMotorPower);
//                robot.rightMotor.setPower(rightMotorPower);
//            if(gamepad1.a)
//            {
//                robot.rightShooter.setPower(0.1);
//                robot.leftShooter.setPower(0.1);
//            }
//            if(gamepad1.b)
//            {
//                robot.rightShooter.setPower(0);
//                robot.leftShooter.setPower(0);
//            }

            robot.waitForTick(40);
        }
    }
}
