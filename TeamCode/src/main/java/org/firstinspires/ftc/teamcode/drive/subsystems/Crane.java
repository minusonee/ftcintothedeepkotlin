package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Crane {

    private static final double POWER = 0.6;

    public double manualTarget = 0;
    //BACK POV
    public DcMotor motorCraneLeft, motorCraneRight;

    public Servo servoGrippy1, servoGrippy2;
    public Servo servoAngle1, servoAngle2;
    public CRServo servoSlide1, servoSlide2;

    public Crane(HardwareMap hardwareMap){
        motorCraneLeft = hardwareMap.dcMotor.get("motorCraneLeft");
        motorCraneRight = hardwareMap.dcMotor.get("motorCraneRight");

        servoGrippy1 = hardwareMap.servo.get("servoGrippy1");
        servoGrippy2 = hardwareMap.servo.get("servoGrippy2");
        servoAngle1 = hardwareMap.servo.get("servoAngle1");
        servoAngle2 = hardwareMap.servo.get("servoAngle2");
        servoSlide1 = hardwareMap.crservo.get("servoSlide1");
        servoSlide2 = hardwareMap.crservo.get("servoSlide2");

        motorCraneLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorCraneLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorCraneLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorCraneRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorCraneRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorCraneRight.setDirection(DcMotorSimple.Direction.FORWARD);

        servoGrippy1.setDirection(Servo.Direction.FORWARD);
        servoGrippy2.setDirection(Servo.Direction.REVERSE);
        servoAngle1.setDirection(Servo.Direction.FORWARD);
        servoAngle2.setDirection(Servo.Direction.REVERSE);
        servoSlide1.setDirection(CRServo.Direction.FORWARD);
        servoSlide2.setDirection(CRServo.Direction.REVERSE);
    }

    public void manualLevel(double manualTarget) {
        motorCraneLeft.setTargetPosition((int) manualTarget);
        motorCraneRight.setTargetPosition((int) manualTarget);
        motorCraneLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorCraneRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(motorCraneLeft.getCurrentPosition() < manualTarget )
        {
            motorCraneLeft.setPower(POWER);
        }
        else{
            motorCraneLeft.setPower(-POWER);
        }
        if(motorCraneRight.getCurrentPosition() < manualTarget )
        {
            motorCraneRight.setPower(POWER);
        }
        else{
            motorCraneRight.setPower(-POWER);
        }
    }

    public void moveSlides(double pos){
        servoSlide1.setPower(pos);
        servoSlide2.setPower(pos);
    }

    public void setGripper(double pos){
        servoGrippy1.setPosition(pos);
        servoGrippy2.setPosition(pos);
    }

    public void setServoAngle(double pos){
        servoAngle1.setPosition(pos);
        servoAngle2.setPosition(pos);
    }
}
