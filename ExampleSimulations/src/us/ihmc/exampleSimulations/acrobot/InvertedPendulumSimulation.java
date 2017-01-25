package us.ihmc.exampleSimulations.acrobot;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

public class InvertedPendulumSimulation
{
   public static final double DT = 0.00001;
   private SimulationConstructionSet sim;

   public InvertedPendulumSimulation()
   {
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      InvertedPendulumRobot robot = new InvertedPendulumRobot();

      robot.setController(new InvertedPendulumController(robot, yoGraphicsListRegistry));

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(32000);

      sim = new SimulationConstructionSet(robot, parameters);
      sim.setDT(DT, 20);
      sim.setGroundVisible(true);
      sim.setCameraPosition(0, -9.0, 0.6);
      sim.setCameraFix(0.0, 0.0, 0.70);
      sim.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      sim.setSimulateDuration(1.0);

      Thread myThread = new Thread(sim);
      myThread.start();
   }

   public static void main(String[] args)
   {
      new InvertedPendulumSimulation();
   }
}
