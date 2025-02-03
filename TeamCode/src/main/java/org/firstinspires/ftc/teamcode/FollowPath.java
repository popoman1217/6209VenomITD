package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Scanner;

public class FollowPath
{
    double prevTime = 0;
    // HACK
    double curAngle = 0;
    //HACK
    double prevAngle = 0;
    // HACK
    double lastAngleDiff = 0;
    double progress = 0;
    public Vector2 arrow;

    double degreesToRadians = 0.017453292519943295;
    double radiansToDegrees = 57.29577951308232;

    String fileName = "/sdcard/FIRST/PathTest.txt";
    File file;
    Scanner scan;

    // This is the previous position of the robot the frame before (distance along the path)
    double[] prevIntersect;

    // This is effectively where you are on the path (including negative position if you haven't got to the starting point)
    public double curPos = 0;
    public double prevPos = 0;

    // This is the last way point you are AT OR PASSED (starts at 0)
    int curWP = 0;
    public double lookAheadDist = 10;
    private double combinedDist = 0;
    ArrayList<ArrayList<Double>> dydxs;

    Vector2 fieldPos = new Vector2(0,0);

    // distance from last way point to this one (wp 0 is 0)
    ArrayList<ArrayList<Double>> dists;

    // total distance up TO that way point. (wp 0 is 0)
    ArrayList<ArrayList<Double>> totalDists;
    ArrayList<ArrayList<Vector2>> wayPointPoss;
    ArrayList<ArrayList<Double>> thetas;


    // These are all the lists of the current trajectory it is on.
    ArrayList<Double> curDydxs;
    ArrayList<Double> curDists;
    ArrayList<Double> curTotalDists;
    ArrayList<Vector2> curWayPointPoss;
    ArrayList<Double> curThetas;

    // The current trajectory number it is on.
    int TrajNumber = 0;

    // Just the marker for the simulation.
    //public Vector2 markerTransform;
    // This is the variable used for the simulation which the only reason I am instantiating this/creating an object
    // is because I need to get the robot poisition which I will do with the localizer. This part on the robot will
    // be a reference to the localizer.
    public RRLocalizationRead posReader;

    // This is the number of inches the robot is from the trajectory before the power to the tangent and perpendicular is .5 each.
    public double TRAJ_WEIGHT_CONST = 5;

    // All for speed control.
    // This is the number of inches the robot is from the end of the trajectory before it starts to slow down.
    double robotSpeed = 0;
    public double ENDPT_CONST = 20;
    public double PCONST = .5;
    public double DCONST = .1;
    double prevError = 0;
    boolean activeFollower = false;
    int trajectoryNumber = 0;

    OpMode master;



    // Start is called before the first frame update
    void Start(OpMode opMode, RRLocalizationRead rr, String m_fileName)
    {
        // posReader is the reference to our localization reader (whether it be Lucca's or RR) that has a returnPos that is a pose2d.
        // That way we dont have to worry about which localizer it is, the RRLocalizationRead class will handle that.
        posReader = rr;

        master = opMode;

        fileName = m_fileName;
        // Just parses through the file and sets each waypoint's (starting at wp0) data to the various arrays
        File file = new File(fileName);
        try{
            scan = new Scanner(file);
        }
        catch (FileNotFoundException e)
        {
            master.telemetry.addData("error", e);
        }
        activeFollower = true;

        // Initializes the ArrayLists of data and adds a blank list.
        dydxs = new ArrayList<>();
        dists = new ArrayList<>();
        totalDists = new ArrayList<>();
        wayPointPoss = new ArrayList<>();
        thetas = new ArrayList<>();

        dydxs.add(new ArrayList<Double>());
        dists.add(new ArrayList<Double>());
        totalDists.add(new ArrayList<Double>());
        wayPointPoss.add(new ArrayList<Vector2>());
        thetas.add(new ArrayList<Double>());

        curWP = 0;
        curPos = 0;
        prevPos = 0;
        prevError = 0;
        combinedDist = 0;

        // This is the local variable that is the current trajectory of the path that it is dealing with in the file.
        int curTraj = 0;

        // Going through and parsing the file
        while (scan.hasNextLine())
        {
            String curString = scan.nextLine();
            //System.out.println(curString);
            // If it reaches a BREAK in the file, it creates the current trajectory to increase and to add a new list of points to the lists.
            if (curString.equals("BREAK"))
            {
                if (scan.hasNext())
                {
                    dydxs.add(new ArrayList<Double>());
                    dists.add(new ArrayList<Double>());
                    totalDists.add(new ArrayList<Double>());
                    wayPointPoss.add(new ArrayList<Vector2>());
                    thetas.add(new ArrayList<Double>());
                    curTraj++;
                }
                continue;
            }

            // This breaks the current line it is on into each of the different data
            Scanner lineScanner = new Scanner(curString).useDelimiter("; ");

            while (lineScanner.hasNext()) {
                String data = lineScanner.next().trim();

                // Parse data based on its key
                if (data.startsWith("dYdX:")) {
                    dydxs.get(curTraj).add(Double.parseDouble(data.split(":")[1]));
                } else if (data.startsWith("totalDist:")) {
                    totalDists.get(curTraj).add(Double.parseDouble(data.split(":")[1]));
                } else if (data.startsWith("dist:")) {
                    dists.get(curTraj).add(Double.parseDouble(data.split(":")[1]));
                } else if (data.startsWith("theta:")) {
                    thetas.get(curTraj).add(Double.parseDouble(data.split(":")[1]));
                } else if (data.startsWith("fieldPos:")) {
                    String vectorData = data.split(":")[1].trim();
                    // Extract the x and y values from the parentheses
                    vectorData = vectorData.substring(1, vectorData.length() - 2); // Remove parentheses
                    String[] coordinates = vectorData.split(", ");
                    double x = Double.parseDouble(coordinates[0]);
                    double y = Double.parseDouble(coordinates[1]);
                    wayPointPoss.get(curTraj).add(new Vector2(x, y));
                }
            }

            lineScanner.close();
        }

        scan.close();

        // Setting the current trajectory data to these lists here.
        curDydxs = dydxs.get(trajectoryNumber);
        curDists = dists.get(trajectoryNumber);
        curTotalDists = totalDists.get(trajectoryNumber);
        curWayPointPoss = wayPointPoss.get(trajectoryNumber);
        curThetas = thetas.get(trajectoryNumber);

        Vector2 initialPos;
        try {
            initialPos = new Vector2(curWayPointPoss.get(0).x, curWayPointPoss.get(0).y);
            prevIntersect = new double[]{initialPos.x, initialPos.y};
            posReader.initLocalization(master.hardwareMap, new Pose2d(initialPos.x, initialPos.y, 0));
        }
        catch (Exception e) {
            prevIntersect = new double[]{0, 0};
            posReader.initLocalization(master.hardwareMap, new Pose2d(0, 0, 0));
            master.telemetry.addLine("CurWayPointPos on initialization does not have a value at index 0: " + e);
            master.telemetry.update();
        }

            Pose2d pos = posReader.returnPose();
            master.telemetry.addLine("fieldPos: x: " + pos.position.x + ", y: " + pos.position.y + ", heading: " + pos.heading);
            fieldPos = new Vector2(pos.position.x, pos.position.y);
            curAngle = getWeightedAngle();
    }

    public Vector2 getFieldPos()
    {
        return fieldPos;
    }


    // This is the number of trajectories in the path.
    public int getTotalTrajectories()
    {
        return dydxs.size();
    }

    // This increments the current trajectory it is on, called by the moverobot class (or the robot master in android studio)
    public void incrementTrajNumber()
    {
        // Setting the current trajectory data to these lists here and incrementing the trajectory number.
        trajectoryNumber++;
        curDydxs = dydxs.get(trajectoryNumber);
        curDists = dists.get(trajectoryNumber);
        curTotalDists = totalDists.get(trajectoryNumber);
        curWayPointPoss = wayPointPoss.get(trajectoryNumber);
        curThetas = thetas.get(trajectoryNumber);
        activeFollower = true;
        curWP = 0;
        curPos = 0;
        prevPos = 0;
        prevError = 0;
        combinedDist = 0;
        prevIntersect = new double[]{wayPointPoss.get(trajectoryNumber).get(0).x, wayPointPoss.get(trajectoryNumber).get(0).y};
        lookAheadDist = 10;
        combinedDist = 0;
        curAngle = getWeightedAngle();
    }

    // Based on the current position of the robot, get the number of waypoints that are between the robot and the max
    // look ahead point.
    private int getNumWPsLookahead()
    {
        calcProgress();
        combinedDist = 0;
        int numWPs = 0;
        if (curPos > curTotalDists.get(curTotalDists.size() - 1))
        {
            combinedDist = curPos - curTotalDists.get(curTotalDists.size() - 1);
            return 0;
        }
        for (int i = curWP; i < curTotalDists.size() - 1; i++)
        {
            if (i == curWP)
            {
                combinedDist += curDists.get(i + 1) + curTotalDists.get(i) - curPos;
            }
            else
            {
                combinedDist += curDists.get(i + 1);
            }
            if (combinedDist > lookAheadDist)
            {
                combinedDist = lookAheadDist;
                numWPs++;
                break;
            }
            numWPs++;
        }
        return numWPs;
    }

    private double[] getIntersectOfTajectory(){
    // Creating the localized field position. This is only for the simulation. There will be no "convert to"
    // in the actual robot. This line will just be something like "getPosition".
    Pose2d fieldPos = posReader.returnPose();
    // Using the theta, this is the dydx of the cur line/traj.
    double slope = getSlopeOfGrossTraj();
    // Math to find the x and y intersection of the trajectory.
    if (trajectoryNumber != 0)
    {
        // Debug.Log("greater than 0");
    }
    double[] intersect = new double[2];
    if (curWP > curWayPointPoss.size() - 1)
    {
        //  Debug.Log("ERROR");
    }
    intersect[0] = (fieldPos.position.y - curWayPointPoss.get(curWP).y + fieldPos.position.x / slope + slope * curWayPointPoss.get(curWP).x) / (1/slope + slope);
    intersect[1] = curWayPointPoss.get(curWP).y + slope * (intersect[0] - curWayPointPoss.get(curWP).x);
    return intersect;
}

    private double getSlopeOfGrossTraj()
    {
        double tan = Math.tan((float)(curAngle * degreesToRadians));
        if (Double.isInfinite(tan) || Double.isNaN(tan))
            tan = 100;
        int sign = (int)Math.signum(Math.signum(Math.cos((float)curAngle * degreesToRadians)) * Math.signum(Math.sin((float)curAngle * degreesToRadians)));
        if (sign == 0)
            sign = 1;
        double slope = 1 / Math.min(Math.max(Math.abs(tan), .01f), 100) * sign;
        return slope;
    }

    private double getWeightedAngle(){
    // Theta is the variable that is added on to each iteration and will be the final weighted angle.
    // DistRemaining is the ever changing distance the waypoints are from the robot, once DistRemaining exceeds
    // The look ahead max distance (combinedDist), it caps out.
    double theta = 0;
    double distRemaining = 0;

    int lookAheadWPs = getNumWPsLookahead();



    if (curPos > curTotalDists.get(curTotalDists.size() - 1) || !activeFollower)
    {
        activeFollower = false;
        combinedDist = getGeneralDist(curWayPointPoss.get(curWayPointPoss.size() - 1).x - fieldPos.x, fieldPos.y - curWayPointPoss.get(curWayPointPoss.size() - 1).y);
        double diffy = fieldPos.y - curWayPointPoss.get(curWayPointPoss.size() - 1).y;
        double diffx = fieldPos.x - curWayPointPoss.get(curWayPointPoss.size() - 1).x;
        theta = Math.atan2(-diffx, -diffy) * radiansToDegrees;
        return theta;
    }
    else
    {
        // Loops through every waypoint that is within the look ahead distance
        for (int i = curWP; i < curWP + lookAheadWPs; i++)
        {

            // Length of line is literally the length of the line from the robot (if 1st iteration)
            // or this waypoint to the next waypoint.
            double lengthOfLine = curTotalDists.get(i + 1) - curTotalDists.get(i);

            // WP1 and 2 are the distances from the robot to the current waypoint begin and end markers.
            // to the current waypoint begin and end markers.
            double wp1 = curTotalDists.get(i) - curPos;
            double wp2 = curTotalDists.get(i + 1) - curPos;
            double deltaTheta = doGimbleCalc(curAngle, curThetas.get(i));

            // If it's the first waypoint, the first waypoint distance is 0.
            if (i == curWP)
            {
                wp1 = 0;
                lengthOfLine = curTotalDists.get(i + 1) - curPos;
            }

            // Capping the length of the line if it is greater than the look ahead distance.
            if (distRemaining + lengthOfLine > combinedDist)
            {
                lengthOfLine = combinedDist - distRemaining;
                distRemaining = combinedDist;
                wp2 = combinedDist;
            }
            else
            {
                // If the line isn't too big/exceeds look ahead, it adds the length of the current line
                // between waypoints to the distance from the robot.
                distRemaining += lengthOfLine;
            }

            // Doing the area of the trapezoid that encompases this area of the angle.
            theta += lengthOfLine * deltaTheta * (((combinedDist - wp1) + (combinedDist - wp2)) / 2.0);
        }

        theta = theta / (Math.pow(combinedDist, 2) / 2.0);
        if (theta + curAngle > 180)
            return theta + curAngle - 360;
        else if (theta + curAngle < -180)
            return theta + curAngle + 360;
        else
            return theta + curAngle;
    }
}

    // Simple gimble calculation (returns difference in angles) where clockwise change is positive.
    private double doGimbleCalc(double prevAngle, double curAngle)
    {
        if (curAngle - prevAngle > 180)
            return curAngle - prevAngle - 360;
        if (curAngle - prevAngle < -180)
            return curAngle - prevAngle + 360;
        return curAngle - prevAngle;
    }


    public double speedController()
    {
        if (curPos >= curTotalDists.get(curTotalDists.size() - 1) - ENDPT_CONST || !activeFollower)
        {
            double PIDVal;
            double multiplier = Math.abs(combinedDist / ENDPT_CONST * PCONST);
            double error = getGeneralDist(curWayPointPoss.get(curWayPointPoss.size() - 1).x - fieldPos.x, curWayPointPoss.get(curWayPointPoss.size() - 1).y - fieldPos.y);
            double d = 0;
            if (prevError != 0)
            {
              //  d = (error - prevError) / Time.deltaTime * DCONST;
            }
            PIDVal = multiplier * error + d * Math.signum(error);
            prevError = error;
            robotSpeed = PIDVal;
        }
        else
        {
            robotSpeed = 1;
        }
        return robotSpeed;
        // My robot gets to a point which is within 2 inches or over the max distance and it starts to slow down based solely off of the trajectory
        // from it and the final position and PID control. How do I do that.
    }

    // Calculates the current waypoint it is at depending on the curPos which is the perpendicular line interseciton with the current
    // trajectory.
    private void calcProgress() {
    progress = curPos / curTotalDists.get(curTotalDists.size() - 1);
    if (prevPos <= curPos)
    {
        for (int wp = curWP; wp < curTotalDists.size(); wp++) {
            if (curTotalDists.get(wp) > curPos)
            {
                curWP = Math.max(wp - 1, 0);
                break;
            }
        }
    }
    else
    {
        for (int wp = curWP; wp > -1; wp--) {
            if (curTotalDists.get(wp) < curPos)
            {
                curWP = wp;
                break;
            }
        }
    }
}

    private double getDistToWP(double[] intersect)
    {
        return Math.sqrt(Math.pow(intersect[0] - curWayPointPoss.get(curWP).x, 2) + Math.pow(intersect[1] - curWayPointPoss.get(curWP).y, 2));
    }

    private void updatePos()
    {
        Pose2d pos = posReader.returnPose();
        fieldPos = new Vector2(pos.position.x, pos.position.y);
        double cos = Math.cos((float)curAngle * degreesToRadians);
        double[] curIntersect = getIntersectOfTajectory();
        if (prevIntersect != null)
        {
            prevPos = curPos;
            if (cos == 0)
            {
                if (curAngle > 0)
                    curPos += Math.sqrt(Math.pow(curIntersect[0] - prevIntersect[0], 2) + Math.pow(curIntersect[1] - prevIntersect[1], 2)) * Math.signum((float)curIntersect[0] - (float)prevIntersect[0]);
                else
                    curPos += Math.sqrt(Math.pow(curIntersect[0] - prevIntersect[0], 2) + Math.pow(curIntersect[1] - prevIntersect[1], 2)) * -Math.signum((float)curIntersect[0] - (float)prevIntersect[0]);
            }
            curPos += Math.sqrt(Math.pow(curIntersect[0] - prevIntersect[0], 2) + Math.pow(curIntersect[1] - prevIntersect[1], 2)) * Math.signum(cos) * Math.signum((float)curIntersect[1] - (float)prevIntersect[1]);
            master.telemetry.addData("curPos", curPos);
        }
        else
            prevIntersect = new double[2];
        // Constantly setting the current angle of the trajector and alligning the arrow accordingly.
        curAngle = getWeightedAngle();
        try{
           // if (!Double.isNaN(curAngle))
                //arrow.transform.localEulerAngles = new Vector3(0, 0, 90f - (float)curAngle);
        }
        catch (Exception e){
    }
        prevIntersect = getIntersectOfTajectory();

    }

    private double getGeneralDist(double val1, double val2)
    {
        return Math.sqrt(Math.pow(val1, 2) + Math.pow(val2, 2));
    }

    public double getSpeedController()
    {
        if (curPos >= curTotalDists.get(curTotalDists.size() - 1) - 15)
        {
            return Math.max(.53, Math.abs(curTotalDists.get(curTotalDists.size() - 1) - curPos) * .05);
        }
        return 1;
    }

    // Gets the normalized vector where x is the normalized x distance, y is the y, and z is the speed.
    public double[] getRobotTrajectory()
    {
        if (activeFollower)
        {
            double slope = getSlopeOfGrossTraj();
            master.telemetry.addData("slope", slope);

            // Just the normalized direction it is supposed to go.
            double lengthOfVector = getGeneralDist(slope, 1);
            int xSign = (int)Math.signum(curAngle);
            int ySign = (int)Math.signum(Math.cos(curAngle * degreesToRadians));
            double[] trajPar = new double[]{1.0 / lengthOfVector * xSign, Math.abs((float)slope) / lengthOfVector * ySign};

            master.telemetry.addData("trajPar", trajPar[0] + " " + trajPar[1]);

            // The normalized distance in the perp direction.
            double[] intersect = getIntersectOfTajectory();
            double[] trajPerp = new double[]{intersect[0] - fieldPos.x, intersect[1] - fieldPos.y};
            master.telemetry.addLine("x: " + fieldPos.x + " y: " + fieldPos.y);
            master.telemetry.addData("intersect", intersect[0] + " " + intersect[1]);
            master.telemetry.addData("trajPerp", trajPerp[0] + " " + trajPerp[1]);
            double lengthOfPerpVector = getGeneralDist(trajPerp[0], trajPerp[1]);
            if (lengthOfPerpVector != 0)
            {
                trajPerp[0] /= lengthOfPerpVector;
                trajPerp[1] /= lengthOfPerpVector;
            }
            else {
                trajPerp[0] = 0;
                trajPerp[1] = 0;
            }


            // gets the distance between the robot and its intersect.
            double dist = lengthOfPerpVector;

            //master.telemetry.addData("dist", dist);

            double powerPerp = dist / (dist + TRAJ_WEIGHT_CONST);
            double powerPar = 1 - powerPerp;

            //master.telemetry.addData("powerPerp", powerPerp);

            double[] finalTraj = new double[]{powerPerp * trajPerp[0] + powerPar * trajPar[0], powerPar * trajPar[1] + powerPerp * trajPerp[1]};
            double lengthOfFinalVector = getGeneralDist(finalTraj[0], finalTraj[1]);
            finalTraj[0] /= lengthOfFinalVector;
            finalTraj[1] /= lengthOfFinalVector;
            return finalTraj;
        }
        else
        {
            double sin = Math.sin((90 + curAngle) * degreesToRadians);
            double cos = Math.cos((90 + curAngle) * degreesToRadians);
            double length = Math.abs(sin) + Math.abs(cos);
            double[] finalTraj = new double[]{-cos / length, sin / length};
            return finalTraj;
        }
    }

    public boolean isAtEnd()
    {
        return getGeneralDist(fieldPos.x - curWayPointPoss.get(curWayPointPoss.size() - 1).x, fieldPos.y - curWayPointPoss.get(curWayPointPoss.size() - 1).y) < 2;//Math.Abs(curPos - curTotalDists[curTotalDists.Count - 1]) <= 1;
    }

    // Update is called once per frame
    public void update() {
        updatePos();
    }
}



        //string path = @"C:\src\ftc\Venom2024-2025IntoTheDeep\Paths\PathTest.txt";
        /*
        Requirements:
        Start the robot and localize its position
        Move along the trajectory based off weighted perpendicular and weighted parallel trajectories
        Determine weighted trajectories based off cur progress
        Determine speed based off of dot product of all the weighted trajs?


        Math for weighted derivative given an array of all the derivatives and their progress to the whole:
        based off of distance to my cur position up to 10 inches?
        so at 10 inches, it doesn't take into account anymore?

        Create const accel. Don't just set power determining velocity

        Different math areas:
        Using localization to determine cur velocity with relation to power and then accel as a result. Using calculated desired speed

        Find weighted traj for desired speed and direction

        Find progress of robot along path

        Localization


        var maxrange = 10 inches
        integral of the dydx * however long it is / max range = weight it gets


            On inititialization, create an array locally of all the points
            current progress
            every frame, find weighted average based on progress
            progress is determined by



        //float slope = -1f /

         */
