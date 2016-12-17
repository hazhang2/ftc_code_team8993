package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hardware5Motors
{
    public DcMotor  leftMotor     = null;
    public DcMotor  rightMotor    = null;
    public DcMotor  leftShooter   = null;
    public DcMotor  rightShooter  = null;
    public DcMotor  ballPicker    = null;
    public Servo    ballPusherServo = null;


    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public Hardware5Motors(){

    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        leftMotor   = hwMap.dcMotor.get("motor_1");
        rightMotor  = hwMap.dcMotor.get("motor_2");

        leftShooter  = hwMap.dcMotor.get("motor_3");
        rightShooter = hwMap.dcMotor.get("motor_4");

        ballPicker   = hwMap.dcMotor.get("motor_5");

        ballPusherServo = hwMap.servo.get("servo_1");


        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        leftShooter.setDirection(DcMotor.Direction.REVERSE);
        rightShooter.setDirection(DcMotor.Direction.FORWARD);

        ballPicker.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        ballPicker.setPower(0);

        leftShooter.setPower(0);
        rightShooter.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ballPicker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        if (remaining > 0)
            Thread.sleep(remaining);

        period.reset();
    }


}

