package us.ihmc.exampleSimulations.simple3dofBiped;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;

public class Simple3dofBipedSimulation
{

   public Simple3dofBipedSimulation()
   {
      Simple3dofBipedRobot simple3dofBiped = new Simple3dofBipedRobot();

      LinearGroundContactModel groundContactModel = new LinearGroundContactModel(simple3dofBiped, simple3dofBiped.getRobotsYoVariableRegistry());
      groundContactModel.setXYStiffness(5000.0);
      groundContactModel.setXYDamping(100.0);
      groundContactModel.setZStiffness(10.0);
      groundContactModel.setZDamping(5.0);
      
      simple3dofBiped.setGroundContactModel(groundContactModel);
      
      SimpleLeapOfFaithHeuristicController controller = new SimpleLeapOfFaithHeuristicController(simple3dofBiped);
      simple3dofBiped.setController(controller);
      
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(4096);

      SimulationConstructionSet scs = new SimulationConstructionSet(simple3dofBiped, parameters);  
      scs.setSimulateNoFasterThanRealTime(true);

      scs.setDT(0.0001, 100);
      scs.setSimulateDuration(200.0);

      scs.startOnAThread();
   }

   public static void main(String[] args)
   {
      new Simple3dofBipedSimulation();
   }
}
