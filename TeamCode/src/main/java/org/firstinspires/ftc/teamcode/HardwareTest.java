package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareTest
{
    public DcMotor  leftMotor     = null;
    public DcMotor  rightMotor    = null;
    public DcMotor  leftShooter   = null;
    public DcMotor  rightShooter  = null;
    public Servo    ballPusherServo = null;

    // public static final double MID_SERVO       =  0.5 ;
    // public static final double ARM_UP_POWER    =  0.45 ;
   // public static final double ARM_DOWN_POWER  = -0.45 ;

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public HardwareTest(){

    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        rightMotor   = hwMap.dcMotor.get("motor_1");
        leftMotor  = hwMap.dcMotor.get("motor_2");

        leftShooter  = hwMap.dcMotor.get("motor_3");
        rightShooter  = hwMap.dcMotor.get("motor_4");

        ballPusherServo = hwMap.servo.get("servo_1");


        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftShooter.setDirection(DcMotor.Direction.REVERSE);
        rightShooter.setDirection(DcMotor.Direction.FORWARD);

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftShooter.setPower(0);
        rightShooter.setPower(0);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

