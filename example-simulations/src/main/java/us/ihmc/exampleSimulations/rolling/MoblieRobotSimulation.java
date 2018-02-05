package us.ihmc.exampleSimulations.rolling;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.exampleSimulations.newtonsCradle.GroundAsABoxRobot;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.robotics.robotDescription.CollisionMasksHelper;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.physics.CollisionHandler;
import us.ihmc.simulationconstructionset.physics.collision.DefaultCollisionHandler;
import us.ihmc.simulationconstructionset.physics.collision.DefaultCollisionVisualizer;

public class MoblieRobotSimulation
{

   public MoblieRobotSimulation()
   {
      System.out.println("Hello World.");
      
      double dt = 0.001;
      
      Vector3D startingPoint = new Vector3D(0.0, 0.0, 1.0);
      
      MobileRobotDescription robotDesription = new MobileRobotDescription("rollingRobot");
      
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(8000);

      SimulationConstructionSet scs = new SimulationConstructionSet(parameters);

      Graphics3DObject staticLinkGraphics = new Graphics3DObject();
      staticLinkGraphics.addCoordinateSystem(0.1);
      scs.addStaticLinkGraphics(staticLinkGraphics);

      CollisionMasksHelper helper = new CollisionMasksHelper();
      int nextGroupBitMask = helper.getNextGroupBitMask();
      // add group and mask for every links for robot.
      
      Robot robot = scs.addRobot(robotDesription);      
      FloatingJoint floatingJoint = (FloatingJoint) robot.getRootJoints().get(0);

      floatingJoint.setPosition(startingPoint);
      
      // robot controller
      
      // ground
      int estimatedNumberOfContactPoints = 100;
      GroundAsABoxRobot groundAsABoxRobot = new GroundAsABoxRobot();
      groundAsABoxRobot.setEstimatedNumberOfContactPoints(estimatedNumberOfContactPoints);
      groundAsABoxRobot.setAddWalls(false);
      //groundAsABoxRobot.setCollisionGroup(nextGroupBitMask);
      groundAsABoxRobot.setCollisionMask(0xff);

      scs.addRobot(groundAsABoxRobot.createRobot());

      scs.setGroundVisible(false);
      
      // simulate
      DefaultCollisionVisualizer collisionVisualizer = new DefaultCollisionVisualizer(100.0, 100.0, 0.01, scs, 1000);
      double coefficientOfRestitution = 0.3;
      double coefficientOfFriction = 0.7;
      CollisionHandler collisionHandler = new DefaultCollisionHandler(coefficientOfRestitution, coefficientOfFriction);
      scs.initializeCollisionDetectionAndHandling(collisionVisualizer, collisionHandler);

      scs.setDT(dt, 1);
      scs.setFastSimulate(true);
//      //      scs.setGroundVisible(false);

      scs.setCameraFix(0.0, 0.0, 0.8);
      scs.setCameraPosition(0.0, -8.0, 1.4);
      scs.startOnAThread();
   }
   
   public static void main(String[] args)
   {
      new MoblieRobotSimulation();
   }
}
