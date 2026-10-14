package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LaunchController {
    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;
    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;

    ElapsedTime feederTimer = new ElapsedTime();


    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    public void init(HardwareMap hwMap) {
        launchState = LaunchState.IDLE;

        launcher = hwMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hwMap.get(CRServo.class, "left_feeder");
        rightFeeder = hwMap.get(CRServo.class, "right_feeder");

    }
    private void set_servo_speed(double speed){
        leftFeeder.setPower(speed);
        rightFeeder.setPower(speed);
    }
    public void stop_all(){
        set_servo_speed(STOP_SPEED);
        launcher.setVelocity(0);
        launchState = LaunchState.IDLE;
    }
    public String getLaunchState() {
        if (launchState == LaunchState.IDLE){
            return "IDLE";
        }
        else if (launchState == LaunchState.SPIN_UP){
            return "SPIN_UP";
        }
        else if (launchState == LaunchState.LAUNCH){
            return "LAUNCH";
        }
        else if (launchState == LaunchState.LAUNCHING){
            return "LAUNCHING";
        }
        else {
            return "UNKNOWN";
        }
    }

    public double launcherVel(){
        return launcher.getVelocity();
    }

    public double servoSpeed(int servo){
        if (servo == 0){
            return (leftFeeder.getPower()+rightFeeder.getPower())/2;
        }
        else if (servo == -1){
            return leftFeeder.getPower();
        }
        else if (servo == 1) {
            return rightFeeder.getPower();
        }
        else{
            return Double.NaN;
        }
    }

    public double servoSpeed(){
        return servoSpeed(0);
    }

    public void launch(boolean shotRequested){
        switch (launchState){
            case IDLE:
                if (shotRequested){
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity()>=LAUNCHER_MIN_VELOCITY){
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                set_servo_speed(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds()>=FEED_TIME_SECONDS){
                    launchState = LaunchState.IDLE;
                    stop_all();
                }
                break;
        }
    }

}
