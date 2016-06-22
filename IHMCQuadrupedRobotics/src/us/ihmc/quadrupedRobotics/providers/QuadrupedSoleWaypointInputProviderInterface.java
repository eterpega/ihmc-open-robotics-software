package us.ihmc.quadrupedRobotics.providers;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.ArrayList;

/**
 * Created by seanmason on 6/17/16.
 */
public interface QuadrupedSoleWaypointInputProviderInterface
{
   ArrayList<Double> getTimeAtWayPointList(RobotQuadrant quadrant);
   ArrayList<Point3d> getWaypointPositionList(RobotQuadrant quadrant);
   ArrayList<Vector3d> getWaypointVelocityList(RobotQuadrant quadrant);
}
