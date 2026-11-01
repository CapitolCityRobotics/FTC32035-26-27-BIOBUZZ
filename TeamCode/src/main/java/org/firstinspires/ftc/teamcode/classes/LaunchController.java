package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LaunchController {
    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double FEED_TIME_SECONDS_FULL = 2;
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;
    final double LAUNCHER_TARGET_VELOCITY = 1150;
    final double LAUNCHER_MIN_VELOCITY = 1100;

    ElapsedTime feederTimer = new ElapsedTime();


    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;
    private DcMotorEx launcher = null;
    private CRServo feeder = null;

    public void init(HardwareMap hwMap) {
        launchState = LaunchState.IDLE;

        launcher = hwMap.get(DcMotorEx.class, "launcher");
        feeder = hwMap.get(CRServo.class, "feeder");

    }
    private void servo_speed(double speed){
        feeder.setPower(speed);
    }
    public void stop_all(){
        servo_speed(STOP_SPEED);
        launcher.setVelocity(STOP_SPEED);
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

    public double servoSpeed(){
            return feeder.getPower();
        }


    public void launch(boolean shotRequested, double servo_speed, boolean all){
        if (servo_speed < -0.1 || 0.1 < servo_speed){
            servo_speed(servo_speed);
        } else if (launchState != LaunchState.LAUNCH && launchState != LaunchState.LAUNCHING){
            servo_speed(0);
        }
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
                if (servo_speed<0.1 && -0.1<servo_speed) {
                    servo_speed(FULL_SPEED);
                }
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds()>=(all ? FEED_TIME_SECONDS_FULL:FEED_TIME_SECONDS)){
                    launchState = LaunchState.IDLE;
                    stop_all();
                }
                break;
        }
    }
}
