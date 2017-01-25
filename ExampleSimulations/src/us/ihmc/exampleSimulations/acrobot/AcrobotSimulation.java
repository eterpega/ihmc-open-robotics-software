package us.ihmc.exampleSimulations.acrobot;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

public class AcrobotSimulation
{
   public static final double DT = 0.000001;
   private SimulationConstructionSet sim;

   public AcrobotSimulation()
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      AcrobotRobot robot = new AcrobotRobot();
      robot.setController(new AcrobotController(robot, yoGraphicsListRegistry));

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(32000);

      sim = new SimulationConstructionSet(robot, parameters);
      YoGraphicReferenceFrame worldFrame = new YoGraphicReferenceFrame(ReferenceFrame.getWorldFrame(), sim.getRootRegistry(), 0.4);
      yoGraphicsListRegistry.registerYoGraphic("world", worldFrame);

      sim.setDT(DT, 50);
      sim.setGroundVisible(false);
      sim.setCameraPosition(0, -9.0, 0.6);
      sim.setCameraFix(0.0, 0.0, 0.70);
      sim.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      sim.setSimulateDuration(60.0);

      Thread myThread = new Thread(sim);
      myThread.start();
   }

   public static void main(String[] args)
   {
      new AcrobotSimulation();
   }
}
