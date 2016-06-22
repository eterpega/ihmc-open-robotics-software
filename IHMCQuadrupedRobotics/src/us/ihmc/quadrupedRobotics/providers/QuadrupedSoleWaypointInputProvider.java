package us.ihmc.quadrupedRobotics.providers;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.quadrupedRobotics.communication.packets.*;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Created by seanmason on 6/17/16.
 */
public class QuadrupedSoleWaypointInputProvider implements QuadrupedSoleWaypointInputProviderInterface
{

   private final AtomicReference<QuadrupedPointListPacket> soleQuadrupedPositionListPacket;
   private final AtomicReference<QuadrupedVectorListPacket> soleQuadrupedVelocityListPacket;
   private final AtomicReference<QuadrupedDoubleListPacket> soleQuadrupedTimingListPacket;

   private final DoubleYoVariable yoHindRightSoleDesiredEndPositionX;
   private final DoubleYoVariable yoHindRightSoleDesiredEndPositionY;
   private final DoubleYoVariable yoHindRightSoleDesiredEndPositionZ;

   private final DoubleYoVariable yoHindLeftSoleDesiredEndPositionX;
   private final DoubleYoVariable yoHindLeftSoleDesiredEndPositionY;
   private final DoubleYoVariable yoHindLeftSoleDesiredEndPositionZ;

   private final DoubleYoVariable yoFrontRightSoleDesiredEndPositionX;
   private final DoubleYoVariable yoFrontRightSoleDesiredEndPositionY;
   private final DoubleYoVariable yoFrontRightSoleDesiredEndPositionZ;

   private final DoubleYoVariable yoFrontLeftSoleDesiredEndPositionX;
   private final DoubleYoVariable yoFrontLeftSoleDesiredEndPositionY;
   private final DoubleYoVariable yoFrontLeftSoleDesiredEndPositionZ;

   private final DoubleYoVariable yoSoleQuadrupedTimingEnd;
   private Integer numberOfWaypoints;

   QuadrupedSoleWaypointInputProvider(GlobalDataProducer globalDataProducer, YoVariableRegistry registry)
   {
      soleQuadrupedPositionListPacket = new AtomicReference<>(new QuadrupedPointListPacket());
      soleQuadrupedVelocityListPacket = new AtomicReference<>(new QuadrupedVectorListPacket());
      soleQuadrupedTimingListPacket = new AtomicReference<>(new QuadrupedDoubleListPacket());

      yoHindRightSoleDesiredEndPositionX= new DoubleYoVariable("HindRightSoleDesiredEndPositionX", registry);
      yoHindRightSoleDesiredEndPositionY= new DoubleYoVariable("HindRightSoleDesiredEndPositionY", registry);
      yoHindRightSoleDesiredEndPositionZ= new DoubleYoVariable("HindRightSoleDesiredEndPositionZ", registry);

      yoHindLeftSoleDesiredEndPositionX= new DoubleYoVariable("HindLeftSoleDesiredEndPositionX", registry);
      yoHindLeftSoleDesiredEndPositionY= new DoubleYoVariable("HindLeftSoleDesiredEndPositionY", registry);
      yoHindLeftSoleDesiredEndPositionZ= new DoubleYoVariable("HindLeftSoleDesiredEndPositionZ", registry);

      yoFrontRightSoleDesiredEndPositionX= new DoubleYoVariable("FrontRightSoleDesiredEndPositionX", registry);
      yoFrontRightSoleDesiredEndPositionY= new DoubleYoVariable("FrontRightSoleDesiredEndPositionY", registry);
      yoFrontRightSoleDesiredEndPositionZ= new DoubleYoVariable("FrontRightSoleDesiredEndPositionZ", registry);

      yoFrontLeftSoleDesiredEndPositionX= new DoubleYoVariable("FrontLeftSoleDesiredEndPositionX", registry);
      yoFrontLeftSoleDesiredEndPositionY= new DoubleYoVariable("FrontLeftSoleDesiredEndPositionY", registry);
      yoFrontLeftSoleDesiredEndPositionZ= new DoubleYoVariable("FrontLeftSoleDesiredPositionZ", registry);

      yoSoleQuadrupedTimingEnd = new DoubleYoVariable("yoSoleQuadrupedTimingEnd", registry);

      if (globalDataProducer != null)
      {
         globalDataProducer.attachListener(QuadrupedPointListPacket.class, new PacketConsumer<QuadrupedPointListPacket>()
         {
            @Override public void receivedPacket(QuadrupedPointListPacket packet)
            {
               soleQuadrupedPositionListPacket.set(packet);
               //UPDATE YO VARIABLES!
               yoFrontLeftSoleDesiredEndPositionX.set(soleQuadrupedPositionListPacket.get().getEndPoint(RobotQuadrant.FRONT_LEFT).getX());
               yoFrontLeftSoleDesiredEndPositionY.set(soleQuadrupedPositionListPacket.get().getEndPoint(RobotQuadrant.FRONT_LEFT).getY());
               yoFrontLeftSoleDesiredEndPositionZ.set(soleQuadrupedPositionListPacket.get().getEndPoint(RobotQuadrant.FRONT_LEFT).getZ());

               yoFrontRightSoleDesiredEndPositionX.set(soleQuadrupedPositionListPacket.get().getEndPoint(RobotQuadrant.FRONT_RIGHT).getX());
               yoFrontRightSoleDesiredEndPositionY.set(soleQuadrupedPositionListPacket.get().getEndPoint(RobotQuadrant.FRONT_RIGHT).getY());
               yoFrontRightSoleDesiredEndPositionZ.set(soleQuadrupedPositionListPacket.get().getEndPoint(RobotQuadrant.FRONT_RIGHT).getZ());

               yoHindLeftSoleDesiredEndPositionX.set(soleQuadrupedPositionListPacket.get().getEndPoint(RobotQuadrant.HIND_LEFT).getX());
               yoHindLeftSoleDesiredEndPositionY.set(soleQuadrupedPositionListPacket.get().getEndPoint(RobotQuadrant.HIND_LEFT).getY());
               yoHindLeftSoleDesiredEndPositionZ.set(soleQuadrupedPositionListPacket.get().getEndPoint(RobotQuadrant.HIND_LEFT).getZ());

               yoHindRightSoleDesiredEndPositionX.set(soleQuadrupedPositionListPacket.get().getEndPoint(RobotQuadrant.HIND_RIGHT).getX());
               yoHindRightSoleDesiredEndPositionY.set(soleQuadrupedPositionListPacket.get().getEndPoint(RobotQuadrant.HIND_RIGHT).getY());
               yoHindRightSoleDesiredEndPositionZ.set(soleQuadrupedPositionListPacket.get().getEndPoint(RobotQuadrant.HIND_RIGHT).getZ());

               yoSoleQuadrupedTimingEnd.set(soleQuadrupedTimingListPacket.get().getEndDouble(RobotQuadrant.FRONT_LEFT));
            }
         });
         globalDataProducer.attachListener(QuadrupedVectorListPacket.class, new PacketConsumer<QuadrupedVectorListPacket>()
         {
            @Override public void receivedPacket(QuadrupedVectorListPacket packet)
            {
               soleQuadrupedVelocityListPacket.set(packet);
               //UPDATE YO VARIABLES!
            }
         });
         globalDataProducer.attachListener(QuadrupedDoubleListPacket.class, new PacketConsumer<QuadrupedDoubleListPacket>()
         {
            @Override public void receivedPacket(QuadrupedDoubleListPacket packet)
            {
               soleQuadrupedTimingListPacket.set(packet);
               //UPDATE YO VARIABLES!
            }
         });

      }
      numberOfWaypoints = soleQuadrupedPositionListPacket.get().getQuadrant(RobotQuadrant.FRONT_LEFT).size();
   }

   public ArrayList<Double> getTimeAtWayPointList(RobotQuadrant quadrant)
   {
      return soleQuadrupedTimingListPacket.get().getQuadrant(quadrant);
   }

   public ArrayList<Point3d> getWaypointPositionList(RobotQuadrant quadrant)
   {
      return soleQuadrupedPositionListPacket.get().getQuadrant(quadrant);
   }

   public ArrayList<Vector3d> getWaypointVelocityList(RobotQuadrant quadrant)
   {
      return soleQuadrupedVelocityListPacket.get().getQuadrant(quadrant);
   }

   public Double getTimeAtWayPoint(RobotQuadrant quadrant, int i)
   {
      return soleQuadrupedTimingListPacket.get().getDouble(quadrant, i);
   }

   public Point3d getWaypointPosition(RobotQuadrant quadrant, int i)
   {
      return soleQuadrupedPositionListPacket.get().getPoint(quadrant, i);
   }

   public Vector3d getWaypointVelocity(RobotQuadrant quadrant, int i)
   {
      return soleQuadrupedVelocityListPacket.get().getVector(quadrant, i);
   }

   public int getNumberOfWaypoints()
   {
      return numberOfWaypoints;
   }
}
