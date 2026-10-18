package org.firstinspires.ftc.teamcode.opmodes;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Autonomous drive program for the robot to find and collect nectar/pollen

 *   drive(inches, power)              forward is positive backward is negative
 *   strafe(inches, power)             right is positive, left is negative
 *   turnTo(degrees)                   field heading; 0 = the way the robot faced at start,
 *                                     positive = counterclockwise (left)
 *   driveUntilBall(maxInches, power)  creep forward watching the color sensor; if a
 *                                     ball shows up, stop and run the intake sequence
 *   intakeAndLaunch()                 the intake sequence by itself: flywheel on,
 *                                     intake on, push into the ball, launch, all off
 * The flywheel only spins during the intake and launch part
 * Hardware this expects
 *   REV Color Sensor V3 on the front, pointing at where a ball would be
 *   continuous-rotation servo turning the rod of compliant wheels (the intake)
 *   motor spinning the flywheel at the back (the launcher)
 */
@Autonomous(name = "Mecanum Auto", group = "Auto")
public class MecanumAuto extends LinearOpMode {

    // SETTINGS: Tune these for our robo


    // Encoder counts per one turn of the motor's output shaft.
    // 537.7 = goBILDA 5203 312 RPM. REV HD Hex 20:1 = 560. Check your motor's spec sheet.
    static final double COUNTS_PER_MOTOR_REV = 537.7;

    // 1.0 if the wheel is mounted straight on the motor shaft. If you have
    // external gears/belts, this is (wheel turns) / (motor turns).
    static final double EXTERNAL_GEAR_RATIO = 1.0;

    // Wheel diameter in inches. 96 mm = 3.78, 104 mm = 4.09, 75 mm = 2.95.
    static final double WHEEL_DIAMETER_INCHES = 4.09;

    // Ticks per wheel revolution divided by inches per revolution (the circumference).
    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * EXTERNAL_GEAR_RATIO) / (WHEEL_DIAMETER_INCHES * Math.PI);

    // Mecanum wheels slip sideways, so strafing covers less ground than the
    // encoders think. Raise this until strafe(24, ...) really moves 24 inches.
    static final double STRAFE_CORRECTION = 1.15;

    // ---- Turning ----

    // Power per degree of heading error. Higher means faster turns means more overshooting.
    static final double TURN_KP = 0.02;
    static final double TURN_MAX_POWER = 0.5;
    // Below this the motors hum but the robot doesn't move.
    static final double TURN_MIN_POWER = 0.12;
    static final double TURN_TOLERANCE_DEG = 1.5;

    // Give up on any single move after this many seconds (e.g. stuck against a wall).
    static final double MOVE_TIMEOUT_SEC = 5.0;

    // ---- Ball detection (color sensor) ----

    // A ball is "in front" when the sensor reads this close or closer, in cm.
    // The Color Sensor V3 only sees about 1 to 10 cm, so stay in that range.
    // To tune: hold a ball where the intake would grab it, read "Sensor distance"
    // on the Driver Hub during INIT, and set this a little above that.
    static final double BALL_DETECT_CM = 5.0;

    // Consecutive "ball" readings required, so one stray reading doesn't trigger the intake.
    static final int BALL_CONFIRM_READINGS = 3;

    // ---- Ball colors (Pollen vs Nectar) ----

    // Placeholder colors not real readings from your balls.
    // Replace with measured values (use the Sensor hue readout on the Driver Hub).
    static final int POLLEN_YELLOW_HEX = 0xFFFF00;   // hue 60
    static final int NECTAR_RED_HEX    = 0xFF0000;   // hue 0
    static final int NECTAR_BLUE_HEX   = 0x0000FF;   // hue 240

    // Max hue difference (degrees around the color wheel) that still counts as a match.
    // Yellow and red are only 60 apart, so keep this under 30 or they'll overlap.
    static final double COLOR_MATCH_TOLERANCE_DEG = 25.0;

    // ---- Intake sequence (servo and flywheel) ----

    // Make negative if the compliant wheels push the ball away and dont pull them in
    static final double INTAKE_POWER = 1.0;

    // Make negative if flywheel spins the wrong way
    static final double FLYWHEEL_POWER = 1.0;
    static final long FLYWHEEL_SPINUP_MS = 800;
    static final double INTAKE_PUSH_INCHES = 4.0;
    static final double INTAKE_PUSH_POWER = 0.25;

    static final long LAUNCH_TIME_MS = 1200;

    // hardware

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    private CRServo intake;                    // rod of compliant wheels on the front
    private DcMotor flywheel;                  // launcher at the back
    private NormalizedColorSensor colorSensor; // ball detection

    enum BallType { POLLEN, NECTAR_RED, NECTAR_BLUE, UNKNOWN }

    private final ElapsedTime timer = new ElapsedTime();

    // =================================================================================
    // MAIN PROGRAM
    // =================================================================================

    @Override
    public void runOpMode() {
        initHardware();

        // Show live sensor readings during INIT so you can check detection before the match.
        while (opModeInInit()) {
            telemetry.addLine("Ready, Press START");
            showSensorTelemetry();
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // ============ YOUR AUTONOMOUS ROUTINE ============
        // Example: look for up to three balls straight ahead. Each driveUntilBall() call
        // runs the full intake sequence before returning if it finds one.
        for (int i = 0; i < 3; i++) {
            boolean foundBall = driveUntilBall(36, 0.3);
            if (!foundBall) break;   // nothing within 36 inches, stop looking
        }

        // Other moves you can mix in:
        // drive(24, 0.5);
        // turnTo(90);
        // strafe(12, 0.4);
        // intakeAndLaunch();   // run the sequence without searching first
        // =================================================

        stopMotors();
        stopSuperstructure();
    }

    // motor setings

    private void initHardware() {
        // Names must match the Driver Hub robot configuration exactly
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // Goal is positive power on every motor drives the robot forward
        // If a wheel spins backward during drive(), flip that motor's direction
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : motors()) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Set these to match which way the REV logo and USB ports face on your robot
        // Wrong values mean wrong turns
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();

        // Must be configured as "Continuous Rotation Servo" not "Servo" or INIT crashes here
        intake = hardwareMap.get(CRServo.class, "intake");

        // The flywheel only needs raw power so no encoder required
        // FLOAT lets it coast down instead of braking a heavy wheel at full speed
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        stopSuperstructure();
    }

    private DcMotor[] motors() {
        return new DcMotor[]{frontLeft, frontRight, backLeft, backRight};
    }

    // DRIVE MOVES

    /** Drive straight. Positive inches = forward. */
    public void drive(double inches, double power) {
        int counts = (int) Math.round(inches * COUNTS_PER_INCH);
        runToPosition(counts, counts, counts, counts, power);
    }

    /** Slide sideways without turning. Positive inches = right. */
    public void strafe(double inches, double power) {
        int counts = (int) Math.round(inches * COUNTS_PER_INCH * STRAFE_CORRECTION);
        // Mecanum strafe right: FL and BR forward, FR and BL backward.
        runToPosition(counts, -counts, -counts, counts, power);
    }

    /**
     * Turn in place to an absolute heading in degrees.
     * 0 = starting direction, positive = counterclockwise.
     */
    public void turnTo(double targetDegrees) {
        for (DcMotor m : motors()) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        timer.reset();
        while (opModeIsActive() && timer.seconds() < MOVE_TIMEOUT_SEC) {
            // Positive error = need to turn left.
            double error = angleWrap(targetDegrees - getHeading());
            if (Math.abs(error) <= TURN_TOLERANCE_DEG) break;

            // Proportional control: power scales with error, clipped to the max,
            // with a floor so the robot doesn't stall just short of the target.
            double power = Range.clip(error * TURN_KP, -TURN_MAX_POWER, TURN_MAX_POWER);
            if (Math.abs(power) < TURN_MIN_POWER) {
                power = Math.signum(power) * TURN_MIN_POWER;
            }

            // Counterclockwise = left side backward, right side forward.
            frontLeft.setPower(-power);
            backLeft.setPower(-power);
            frontRight.setPower(power);
            backRight.setPower(power);

            telemetry.addData("Turning to", "%.1f deg", targetDegrees);
            telemetry.addData("Heading", "%.1f deg", getHeading());
            telemetry.addData("Error", "%.1f deg", error);
            telemetry.update();
        }
        stopMotors();
        sleep(100); // settle before the next move
    }

    // =================================================================================
    // BALL MOVES
    // =================================================================================

    /**
     * Creep forward looking for a ball. If one appears, stop and run the intake sequence.
     * Returns true if a ball was found, false if it drove maxInches without seeing one.
     * Keep power low (0.2 to 0.4) so the robot stops before bumping the ball away.
     */
    public boolean driveUntilBall(double maxInches, double power) {
        if (!opModeIsActive()) return false;

        int startFL = frontLeft.getCurrentPosition();
        int startFR = frontRight.getCurrentPosition();
        int startBL = backLeft.getCurrentPosition();
        int startBR = backRight.getCurrentPosition();
        double maxTicks = Math.abs(maxInches) * COUNTS_PER_INCH;

        boolean found = false;
        int readingsInARow = 0;

        // No target position: we don't know where we'll stop, so just roll and watch.
        for (DcMotor m : motors()) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setPower(Math.abs(power));
        }

        timer.reset();
        while (opModeIsActive()
                && timer.seconds() < MOVE_TIMEOUT_SEC
                && ticksTravelledSince(startFL, startFR, startBL, startBR) < maxTicks) {

            readingsInARow = ballDetected() ? readingsInARow + 1 : 0;
            if (readingsInARow >= BALL_CONFIRM_READINGS) {
                found = true;
                break;
            }

            telemetry.addLine("Looking for a ball...");
            showSensorTelemetry();
            telemetry.update();
        }
        stopMotors();

        if (found && opModeIsActive()) {
            intakeAndLaunch();
        }
        return found;
    }

    /**
     * The intake sequence: spin up the flywheel, pull the ball in, launch it, shut down.
     * The flywheel is on only between the start and end of this method.
     */
    public void intakeAndLaunch() {
        if (!opModeIsActive()) return;

        // Color is only reported for now; every ball gets the same sequence.
        BallType ball = identifyBall();
        telemetry.addData("Ball found!", ball);
        telemetry.addLine("Running intake sequence.");
        telemetry.update();

        // Flywheel first, so it's at speed by the time the ball reaches it.
        flywheel.setPower(FLYWHEEL_POWER);
        sleep(FLYWHEEL_SPINUP_MS);

        // Intake on, then nudge into the ball so the compliant wheels grab it.
        intake.setPower(INTAKE_POWER);
        drive(INTAKE_PUSH_INCHES, INTAKE_PUSH_POWER);

        // Let the ball travel through the robot and out the flywheel.
        sleep(LAUNCH_TIME_MS);

        stopSuperstructure();
    }

    // =================================================================================
    // SENSOR HELPERS
    // =================================================================================

    /** True when something is within BALL_DETECT_CM of the color sensor. */
    // Uses the V3's proximity reading rather than color; it's more reliable for presence.
    private boolean ballDetected() {
        return getSensorDistanceCm() <= BALL_DETECT_CM;
    }

    private double getSensorDistanceCm() {
        // The Color Sensor V3 also implements DistanceSensor.
        return ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
    }

    /** Hue (0 to 360) of what the color sensor sees. */
    private float getSensorHue() {
        NormalizedRGBA reading = colorSensor.getNormalizedColors();
        float[] hsv = new float[3];
        Color.colorToHSV(reading.toColor(), hsv);
        return hsv[0];
    }

    /** Classifies the ball by comparing the sensor's hue with each reference color's hue. */
    // Hue is used instead of raw RGB so the match holds up under dim or uneven lighting.
    private BallType identifyBall() {
        double sensorHue = getSensorHue();

        if (hueDistance(sensorHue, hueOfHex(POLLEN_YELLOW_HEX)) <= COLOR_MATCH_TOLERANCE_DEG) {
            return BallType.POLLEN;
        }
        if (hueDistance(sensorHue, hueOfHex(NECTAR_RED_HEX)) <= COLOR_MATCH_TOLERANCE_DEG) {
            return BallType.NECTAR_RED;
        }
        if (hueDistance(sensorHue, hueOfHex(NECTAR_BLUE_HEX)) <= COLOR_MATCH_TOLERANCE_DEG) {
            return BallType.NECTAR_BLUE;
        }
        return BallType.UNKNOWN;
    }

    private double hueOfHex(int hex) {
        float[] hsv = new float[3];
        Color.colorToHSV(hex, hsv);
        return hsv[0];
    }

    /** Shortest distance between two hues around the color wheel (so 350 is 10 from 0). */
    private double hueDistance(double hueA, double hueB) {
        return Math.abs(angleWrap(hueA - hueB));
    }

    // Adds sensor lines to telemetry; caller is responsible for telemetry.update().
    private void showSensorTelemetry() {
        telemetry.addData("Sensor distance", "%.1f cm", getSensorDistanceCm());
        telemetry.addData("Sensor hue", "%.0f", getSensorHue());
        telemetry.addData("Ball in front?", ballDetected() ? "YES" : "no");
        telemetry.addData("Ball color", identifyBall());
    }

    // =================================================================================
    // MOTOR HELPERS
    // =================================================================================

    private void stopSuperstructure() {
        intake.setPower(0);
        flywheel.setPower(0);   // coasts down (FLOAT)
    }

    /** Average distance the four wheels have turned since the given readings, in ticks. */
    private double ticksTravelledSince(int startFL, int startFR, int startBL, int startBR) {
        int fl = Math.abs(frontLeft.getCurrentPosition() - startFL);
        int fr = Math.abs(frontRight.getCurrentPosition() - startFR);
        int bl = Math.abs(backLeft.getCurrentPosition() - startBL);
        int br = Math.abs(backRight.getCurrentPosition() - startBR);
        return (fl + fr + bl + br) / 4.0;
    }

    /** Move each wheel by the given number of encoder counts, then stop. */
    private void runToPosition(int fl, int fr, int bl, int br, double power) {
        if (!opModeIsActive()) return;

        // Targets are relative to the current position; encoders aren't reset between moves.
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + fl);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + fr);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + bl);
        backRight.setTargetPosition(backRight.getCurrentPosition() + br);

        // In RUN_TO_POSITION, power is a speed limit and direction comes from the target,
        // so power is always positive.
        for (DcMotor m : motors()) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(Math.abs(power));
        }

        // Stop as soon as the first motor arrives so the robot doesn't twist at the end.
        timer.reset();
        while (opModeIsActive()
                && timer.seconds() < MOVE_TIMEOUT_SEC
                && frontLeft.isBusy() && frontRight.isBusy()
                && backLeft.isBusy() && backRight.isBusy()) {
            telemetry.addData("Target", "FL %d  FR %d", frontLeft.getTargetPosition(), frontRight.getTargetPosition());
            telemetry.addData("Now", "FL %d  FR %d", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
            telemetry.update();
        }

        stopMotors();
        for (DcMotor m : motors()) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        sleep(100);
    }

    private void stopMotors() {
        for (DcMotor m : motors()) {
            m.setPower(0);
        }
    }

    /** Current heading in degrees, -180 to 180, positive = counterclockwise. */
    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /** Wrap an angle into -180..180 so the robot always turns the short way. */
    private double angleWrap(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees <= -180) degrees += 360;
        return degrees;
    }
}