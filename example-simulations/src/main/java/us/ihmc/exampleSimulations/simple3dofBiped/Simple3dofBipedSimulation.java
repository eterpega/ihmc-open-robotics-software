package us.ihmc.exampleSimulations.simple3dofBiped;

import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;

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
      
      double rampDownOneStartX = 5.0;
      double rampDownOneEndX = 10.0;
      double rampDownOneBottomZ = -2.0;
      double rampDownTwoEndX = 15.0;
      double rampDownTwoBottomZ = -6.0;
      
      
      double rampUpStartX = 40.0;
      double rampUpEndX = 100.0;
      

      CombinedTerrainObject3D terrainObject = new CombinedTerrainObject3D("Terrain");
      terrainObject.addBox(-3.0, -1.0, rampDownOneStartX, 1.0, -0.01, 0.0);
      terrainObject.addRamp(rampDownOneEndX, -1.0, rampDownOneStartX, 1.0, rampDownOneBottomZ, 0.0, YoAppearance.Red());
      terrainObject.addRamp(rampDownTwoEndX, -1.0, rampDownOneEndX, 1.0, rampDownTwoBottomZ, rampDownOneBottomZ, YoAppearance.Blue());
      
      terrainObject.addBox(rampDownTwoEndX, -1.0, rampUpStartX, 1.0, rampDownTwoBottomZ - 0.01, rampDownTwoBottomZ);
      terrainObject.addRamp(rampUpStartX, -1.0, rampUpEndX, 1.0, rampDownTwoBottomZ, 0.0, YoAppearance.Red());

//      RampTerrainObject downRamp = new RampTerrainObject(10.0, -1.0, 5.0, 1.0, -1.0, 0.0);
      
//      terrainObject.addRamp(3.0, -1.0, 20.0, 1.0, 1.0, YoAppearance.AliceBlue());
//      Vector3D surfaceNormal = new Vector3D(0.5, 0.0, 1.0);
//      Point3D intersectionPoint = new Point3D();
//      double maxXY = 100.0;
//      SlopedPlaneGroundProfile profile3D = new SlopedPlaneGroundProfile(surfaceNormal, intersectionPoint, maxXY);
      groundContactModel.setGroundProfile3D(terrainObject);
      
      simple3dofBiped.setGroundContactModel(groundContactModel);
      
      SimpleLeapOfFaithHeuristicController controller = new SimpleLeapOfFaithHeuristicController(simple3dofBiped);
      simple3dofBiped.setController(controller);
      
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(4096);

      SimulationConstructionSet scs = new SimulationConstructionSet(simple3dofBiped, parameters);  
      scs.setSimulateNoFasterThanRealTime(true);

      scs.setGroundVisible(false);
      scs.addStaticLinkGraphics(terrainObject.getLinkGraphics());
      
      scs.setDT(0.0001, 100);
      scs.setSimulateDuration(200.0);

      scs.startOnAThread();
   }

   public static void main(String[] args)
   {
      new Simple3dofBipedSimulation();
   }
}
