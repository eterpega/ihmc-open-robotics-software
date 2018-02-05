package us.ihmc.llama;

import us.ihmc.euclid.tuple3D.Vector3D;

import us.ihmc.llama.model.LlamaModelFactory;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class LlamaSDFViewer
{
   public static void main(String[] args)
   {
      LlamaModelFactory modelFactory = new LlamaModelFactory();
      FloatingRootJointRobot sdfRobot = new FloatingRootJointRobot(modelFactory.createSdfRobot());
      sdfRobot.setPositionInWorld(new Vector3D(0.0, 0.0, 0.6));
      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot);
      scs.startOnAThread();
   }
}
