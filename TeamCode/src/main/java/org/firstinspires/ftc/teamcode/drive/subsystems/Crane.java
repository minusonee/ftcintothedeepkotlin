package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Crane {

    private static final double POWER = 1;

    public double manualTarget = 0;

    public DcMotor motorCrane1, motorCrane2;
    public Servo servoGrippy, servoAngle, servoSlide1, servoSlide2;

    public Crane(HardwareMap hardwareMap){
        motorCrane1 = hardwareMap.dcMotor.get("motorCrane1");
        motorCrane2 = hardwareMap.dcMotor.get("motorCrane2");

        servoGrippy = hardwareMap.servo.get("servoGrippy");
        servoAngle = hardwareMap.servo.get("servoAngle");
        servoSlide1 = hardwareMap.servo.get("servoSlide1");
        servoSlide2 = hardwareMap.servo.get("servoSlide2");

        motorCrane1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorCrane1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorCrane1.setDirection(DcMotorSimple.Direction.FORWARD);
        motorCrane2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorCrane2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorCrane2.setDirection(DcMotorSimple.Direction.REVERSE);

        servoGrippy.setDirection(Servo.Direction.FORWARD);
        servoAngle.setDirection(Servo.Direction.FORWARD);
        servoSlide1.setDirection(Servo.Direction.FORWARD);
        servoSlide2.setDirection(Servo.Direction.REVERSE);
    }

    public void manualPivot(int power){
        motorCrane1.setPower(power);
        motorCrane2.setPower(power);
    }

    public void moveSlides(double pos){
        servoSlide1.setPosition(pos);
        servoSlide2.setPosition(pos);
    }

    public void setGripper(double pos){
        servoGrippy.setPosition(pos);
    }

    public void setServoAngle(double pos){
        servoAngle.setPosition(pos);
    }
}
