package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "kalman odom", group = "Sensors")
public class kalmanFilterUsage extends LinearOpMode {

    private DcMotorEx verticalOdom, horizontalOdom;
    private IMU imu;

    // variables to track the robot's state
    private double robotX = 0, robotY = 0; // robot's position in inches
    private double velocityX = 0, velocityY = 0; // robot's velocity in inches per second
    private double heading = 0;  // robot's orientation (in radians)
    private double prevVertical = 0, prevHorizontal = 0; // previous encoder positions in inches
    private double prevTime = 0; // previous timestamp in seconds

    // constants for odometry calculations
    private static final double TICKS_PER_REV = 2000; // number of encoder ticks per wheel revolution
    private static final double WHEEL_DIAMETER = 2; // diameter of the odometry wheel in inches
    private static final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER); // encoder ticks per inch traveled

    // kalman filter matrices to track and predict robot's movement
    private double[][] state = {{0}, {0}, {0}, {0}}; // the state: [x, y, velocity_x, velocity_y]
    private double[][] stateCovariance = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}; // uncertainty in the state
    private final double[][] processNoise = {{0.1, 0, 0, 0}, {0, 0.1, 0, 0}, {0, 0, 0.05, 0}, {0, 0, 0, 0.05}}; // noise from movement and model imperfections
    private final double[][] measurementNoise = {{0.2, 0, 0}, {0, 0.2, 0}, {0, 0, 0.1}}; // noise from sensors like encoders and imu

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize the hardware: vertical and horizontal odometry wheels and imu
        verticalOdom = hardwareMap.get(DcMotorEx.class, "verticalOdom");
        horizontalOdom = hardwareMap.get(DcMotorEx.class, "horizontalOdom");
        imu = hardwareMap.get(IMU.class, "imu");

        // reset encoder values to zero
        verticalOdom.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        horizontalOdom.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // reset imu orientation
        imu.resetYaw();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // record the starting time in seconds, only nanoseconds showed up so i just divided by 10^9
        prevTime = System.nanoTime() / 1e9;

        while (opModeIsActive()) {
            // calculate the time elapsed since the last loop iteration
            double currentTime = System.nanoTime() / 1e9;
            double deltaTime = currentTime - prevTime;
            prevTime = currentTime;

            // get the current encoder values (convert from ticks to inches)
            double currentVertical = verticalOdom.getCurrentPosition() / TICKS_PER_INCH;
            double currentHorizontal = horizontalOdom.getCurrentPosition() / TICKS_PER_INCH;

            // get the current heading of the robot from the imu (in radians)
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // calculate how much the encoder values and heading have changed
            double deltaVertical = currentVertical - prevVertical;
            double deltaHorizontal = currentHorizontal - prevHorizontal;
            double deltaHeading = currentHeading - heading;

            // update previous encoder and heading values for the next loop
            prevVertical = currentVertical;
            prevHorizontal = currentHorizontal;
            heading = currentHeading;

            // calculate the robot's movement in x and y based on the encoder changes
            // we use trigonometry to adjust for the robot's heading
            double deltaX = deltaVertical * Math.cos(heading) - deltaHorizontal * Math.sin(heading);
            double deltaY = deltaVertical * Math.sin(heading) + deltaHorizontal * Math.cos(heading);

            // kalman filter prediction step: predict the next state based on current state and velocity
            double[][] predictedState = {
                    {state[0][0] + state[2][0] * deltaTime}, // predicted x position
                    {state[1][0] + state[3][0] * deltaTime}, // predicted y position
                    {state[2][0]}, // x velocity remains the same
                    {state[3][0]}  // y velocity remains the same
            };

            // predict the new uncertainty in the state
            double[][] predictedCovariance = addMatrices(stateCovariance, processNoise);

            // measurement step: based on the actual sensor readings
            double[][] measurement = {
                    {robotX + deltaX}, // measured x position
                    {robotY + deltaY}, // measured y position
                    {heading} // measured heading
            };

            // calculate the kalman gain, which balances the prediction and measurement
            double[][] kalmanGain = calculateKalmanGain(predictedCovariance, measurementNoise);

            // calculate the difference between the prediction and the actual measurement
            double[][] innovation = subtractMatrices(measurement, predictedState);

            // update the state with the kalman gain and the measurement difference
            state = addMatrices(predictedState, multiplyMatrices(kalmanGain, innovation));

            // update the uncertainty in the state
            stateCovariance = multiplyMatrices(
                    subtractMatrices(identityMatrix(4), kalmanGain),
                    predictedCovariance
            );

            // update the robot's position and velocity based on the kalman filter state
            robotX = state[0][0];
            robotY = state[1][0];
            velocityX = state[2][0];
            velocityY = state[3][0];

            // send telemetry data to the driver station for debugging
            telemetry.addData("X Position (in)", robotX);
            telemetry.addData("Y Position (in)", robotY);
            telemetry.addData("X Velocity (in/s)", velocityX);
            telemetry.addData("Y Velocity (in/s)", velocityY);
            telemetry.addData("Heading (deg)", Math.toDegrees(heading));
            telemetry.update();
        }
    }

    // method to add two tables of numbers (matrices)
    private double[][] addMatrices(double[][] a, double[][] b) {
        int rows = a.length; // get the number of rows (how many horizontal lines of numbers) in table a
        int cols = a[0].length; // get the number of columns (how many vertical groups of numbers) in table a
        double[][] result = new double[rows][cols]; // create a new table to store the result

        // loop through each row (horizontal line) and column (vertical group) in both tables
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[i][j] = a[i][j] + b[i][j]; // add the same numbers from both tables and store them in the result
            }
        }
        return result; // return the table with the added numbers
    }

    // method to subtract one table of numbers from another
    private double[][] subtractMatrices(double[][] a, double[][] b) {
        int rows = a.length; // get the number of rows (horizontal lines) in table a
        int cols = a[0].length; // get the number of columns (vertical groups) in table a
        double[][] result = new double[rows][cols]; // create a new table to store the result

        // loop through each row (horizontal line) and column (vertical group) in both tables
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[i][j] = a[i][j] - b[i][j]; // subtract the numbers in the same position and store them in the result
            }
        }
        return result; // return the table with the subtracted numbers
    }

    // method to multiply two tables of numbers together (this is a little more complicated)
    private double[][] multiplyMatrices(double[][] a, double[][] b) {
        int rowsA = a.length; // get the number of rows in table a
        int colsA = a[0].length; // get the number of columns in table a
        int colsB = b[0].length; // get the number of columns in table b
        double[][] result = new double[rowsA][colsB]; // create a new table to store the result

        // loop through each row in table a
        for (int i = 0; i < rowsA; i++) {
            // loop through each column in table b
            for (int j = 0; j < colsB; j++) {
                // multiply numbers in the same position from table a and table b, and add them together
                for (int k = 0; k < colsA; k++) {
                    result[i][j] += a[i][k] * b[k][j]; // multiply and add the numbers to the result table
                }
            }
        }
        return result; // return the table with the multiplied numbers
    }

    // method to calculate something called Kalman Gain (used to help decide how to combine predictions and measurements), if gain is high (close to 1), we trust predictions, if gain is low (close to 0), we trust measurements
    private double[][] calculateKalmanGain(double[][] covariance, double[][] noise) {
        int size = covariance.length; // get the size of the table (it’s square, same number of rows and columns)
        double[][] gain = new double[size][size]; // create a new table to store the Kalman Gain

        // loop through each row and column of the table
        for (int i = 0; i < size; i++) {
            for (int j = 0; j < size; j++) {
                if (i == j) { // if we are on the diagonal (where row and column are the same)
                    // calculate the Kalman Gain for this position
                    gain[i][j] = covariance[i][j] / (covariance[i][j] + noise[i][j]);
                } else {
                    gain[i][j] = 0; // set all other positions to 0
                }
            }
        }
        return gain; // return the table with the Kalman Gain
    }

    // method to create a special table called an "identity matrix" (used in many calculations)
    private double[][] identityMatrix(int size) {
        double[][] identity = new double[size][size]; // create a square table of the given size

        // loop through the table and set the diagonal elements (where row equals column) to 1
        for (int i = 0; i < size; i++) {
            identity[i][i] = 1; // set the diagonal elements to 1
        }

        return identity; // returns the identity table that allows the gain to be assoicated with a couple numbers realtive to was calulated above
    }
}