package us.ihmc.exampleSimulations.stickRobot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.visualizer.CommonInertiaEllipsoidsVisualizer;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;


public class StickRobotSDFLoadingDemo
{
   private static final boolean SHOW_ELLIPSOIDS = false;
   private static final boolean SHOW_COORDINATES_AT_JOINT_ORIGIN = false;

   private SimulationConstructionSet scs;

   public StickRobotSDFLoadingDemo()
   {
      StickRobotModel robotModel = new StickRobotModel(DRCRobotModel.RobotTarget.SCS, false);

      FloatingRootJointRobot stickRobot = robotModel.createHumanoidFloatingRootJointRobot(false);
      stickRobot.setPositionInWorld(new Vector3D());

      if (SHOW_ELLIPSOIDS)
      {
         addIntertialEllipsoidsToVisualizer(stickRobot);
      }

      if (SHOW_COORDINATES_AT_JOINT_ORIGIN)
         addJointAxis(stickRobot);

      FullRobotModel fullRobotModel = robotModel.createFullRobotModel();

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      CommonInertiaEllipsoidsVisualizer inertiaVis = new CommonInertiaEllipsoidsVisualizer(fullRobotModel.getElevator(), yoGraphicsListRegistry);
      inertiaVis.update();


      scs = new SimulationConstructionSet(stickRobot);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setGroundVisible(false);
      scs.startOnAThread();
   }

   private void addIntertialEllipsoidsToVisualizer(FloatingRootJointRobot stickRobot)
   {
      ArrayList<Joint> joints = new ArrayList<>();
      joints.add(stickRobot.getRootJoint());

      HashSet<Link> links = getAllLinks(joints, new HashSet<Link>());

      for (Link l : links)
      {
         AppearanceDefinition appearance = YoAppearance.Green();
         appearance.setTransparency(0.6);
         l.addEllipsoidFromMassProperties(appearance);
         l.addCoordinateSystemToCOM(0.5);
         //         l.addBoxFromMassProperties(appearance);
      }
   }

   private HashSet<Link> getAllLinks(ArrayList<Joint> joints, HashSet<Link> links)
   {
      for (Joint j : joints)
      {
         links.add(j.getLink());

         if (!j.getChildrenJoints().isEmpty())
         {
            links.addAll(getAllLinks(j.getChildrenJoints(), links));
         }
      }

      return links;
   }

   public void addJointAxis(FloatingRootJointRobot valkyrieRobot)
   {

      ArrayList<OneDegreeOfFreedomJoint> joints = new ArrayList<>(Arrays.asList(valkyrieRobot.getOneDegreeOfFreedomJoints()));

      for (OneDegreeOfFreedomJoint joint : joints)
      {
         Graphics3DObject linkGraphics = new Graphics3DObject();
         linkGraphics.addCoordinateSystem(0.5);
         linkGraphics.combine(joint.getLink().getLinkGraphics());
         joint.getLink().setLinkGraphics(linkGraphics);
      }
   }

   public static void main(String[] args)
   {
      new StickRobotSDFLoadingDemo();
   }

}
