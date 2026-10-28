package org.firstinspires.ftc.teamcode.classes;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeController {
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double MAX_SPEED = 1.0;
    final double MAX_REVERSE_SPEED = 0.75;
    private DcMotorEx intake_motor = null;
    private CRServo left_intake = null;
    private CRServo right_intake = null;
    public void init(HardwareMap hwMap) {
        intake_motor = hwMap.get(DcMotorEx.class, "intake motor");
        left_intake = hwMap.get(CRServo.class, "left intake");
        right_intake = hwMap.get(CRServo.class, "right intake");

        left_intake.setDirection(CRServo.Direction.REVERSE);
    }
    public void set_intake_speed(double speed){
        double max_speed;
        if (speed<0) {
            max_speed = MAX_REVERSE_SPEED;
        }
        else if (speed>0){
            max_speed = MAX_SPEED;
        }
        else{
            max_speed = STOP_SPEED;
        }
        intake_motor.setPower(square(speed)*max_speed);
        left_intake.setPower(square(speed)*max_speed);
        right_intake.setPower(square(speed)*max_speed);
    }

    public double get_intake_speed(){
        return ((intake_motor.getPower() + right_intake.getPower() + left_intake.getPower()) / 3);
    }

    public void intake_off() {
        set_intake_speed(STOP_SPEED);
    }

    private double square(double num){
        return num * abs(num);
    }

}
