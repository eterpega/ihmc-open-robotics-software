package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationConstructionSetTools.robotController.ContactController;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.BoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;


public class HatchEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D hatchEnvironment;
   
   private static int NUMBER_OF_HATCHES = 0;                               // Simulation values:
//   private static final double FORWARD_OFFSETS[] = {1.21, 3.71, 6.21, 8.71};     // {1.21, 3.71, 6.21, 8.71};
//   private static final double SIDEWAY_OFFSETS[] = {0.08, 1.08, 0.08, 0.08};     // {0.08, 1.08, 0.08, 0.08};
   
   private static final Point3D ORIGINS[] = {new Point3D(1.00, -0.00, 0.00), new Point3D(5.00, 1.00, 0.00), new Point3D(8.00, -1.00, 0.00), new Point3D(11.00, 0.00, 0.00)};
   private static final double YAWS[] = {0.00, 0.00, 0.00, 0.00};
   private static final double STEP_HEIGHTS[] = {0.16, 0.05, 0.20, 0.10};        // {0.15, 0.05, 0.20, 0.10};
   private static final double OPENING_HEIGHTS[] = {1.64, 1.60, 1.60, 1.60};     // {1.55, 1.55, 1.55, 1.55};
   private static final double OPENING_WIDTHS[] = {0.90, 0.86, 0.86, 0.86};      // {0.86, 0.86, 0.86, 0.86};
   private static final double OPENING_THICKNESSES[] = {0.10, 0.10, 0.03, 0.10}; // {0.12, 0.12, 0.05, 0.12};
   
   private static List<Hatch> HATCHES = new ArrayList<Hatch>();
   
   public HatchEnvironment()
   {
      hatchEnvironment = new CombinedTerrainObject3D(getClass().getSimpleName());
      hatchEnvironment.addTerrainObject(setUpGround("Ground"));
      
      createHatchAndAddItToEnvironment(ORIGINS[0], YAWS[0], STEP_HEIGHTS[0], OPENING_HEIGHTS[0], OPENING_WIDTHS[0], OPENING_THICKNESSES[0]);
      createHatchAndAddItToEnvironment(ORIGINS[1], YAWS[1], STEP_HEIGHTS[1], OPENING_HEIGHTS[1], OPENING_WIDTHS[1], OPENING_THICKNESSES[1]);
      createHatchAndAddItToEnvironment(ORIGINS[2], YAWS[2], STEP_HEIGHTS[2], OPENING_HEIGHTS[2], OPENING_WIDTHS[2], OPENING_THICKNESSES[2]);
      createHatchAndAddItToEnvironment(ORIGINS[3], YAWS[3], STEP_HEIGHTS[3], OPENING_HEIGHTS[3], OPENING_WIDTHS[3], OPENING_THICKNESSES[3]);
   }
   
   private CombinedTerrainObject3D setUpGround(String name)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);
      combinedTerrainObject.addBox(-5.0, -10.0, 15.0, 10.0, -0.05, 0.0, YoAppearance.Gray());

      return combinedTerrainObject;
   }
   
   private void createHatchAndAddItToEnvironment(Point3D origin, double yaw, double stepHeight, double openingHeight, double openingWidth, double openingThickness)
   {
      RigidBodyTransform hatchToWorldTransform = new RigidBodyTransform(new Quaternion(), origin);
      hatchToWorldTransform.appendYawRotation(yaw);
      
      HATCHES.add(new Hatch(hatchToWorldTransform, stepHeight, openingHeight, openingWidth, openingThickness));
      addHatchToEnvironment("Hatch" + NUMBER_OF_HATCHES, HATCHES.get(NUMBER_OF_HATCHES));
      
      NUMBER_OF_HATCHES++;
   }
   
   private void addHatchToEnvironment(String name, Hatch hatch)
   {
      CombinedTerrainObject3D hatchObject = new CombinedTerrainObject3D(name);

      RigidBodyTransform hatchToWorld = hatch.getHatchToWorldTransform();

      hatchObject.addBox(hatchToWorld.getTranslationX(), hatchToWorld.getTranslationY()+(hatch.getWidth()/2.0)+0.5,  hatchToWorld.getTranslationX()+hatch.getThickness(), hatchToWorld.getTranslationY()-(hatch.getWidth()/2.0)-0.5, 0, hatch.getStepHeight(), YoAppearance.DarkGray());

      hatchObject.addBox(hatchToWorld.getTranslationX(),hatchToWorld.getTranslationY()+hatch.getWidth()/2.0,  hatchToWorld.getTranslationX()+hatch.getThickness(),hatchToWorld.getTranslationY()+(hatch.getWidth()/2.0)+0.5, 0, hatch.getOpeningHeight()+hatch.getStepHeight(), YoAppearance.DarkGray());
      hatchObject.addBox(hatchToWorld.getTranslationX(),hatchToWorld.getTranslationY()-hatch.getWidth()/2.0,  hatchToWorld.getTranslationX()+hatch.getThickness(),hatchToWorld.getTranslationY()-(hatch.getWidth()/2.0)-0.5, 0, hatch.getOpeningHeight()+hatch.getStepHeight(), YoAppearance.DarkGray());

      BoxTerrainObject hatchTop = new BoxTerrainObject(hatchToWorld.getTranslationX(),hatchToWorld.getTranslationY()+(hatch.getWidth()/2)+0.5,  hatchToWorld.getTranslationX()+hatch.getThickness(),hatchToWorld.getTranslationY()-(hatch.getWidth()/2.0)-0.5, hatch.getOpeningHeight()+hatch.getStepHeight(), 1.95, YoAppearance.DarkGray());
      hatchObject.addStaticLinkGraphics(hatchTop.getLinkGraphics());
      
      hatchEnvironment.addTerrainObject(hatchObject);
   }
   
   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return hatchEnvironment;
   }

   @Override
   public List<? extends Robot> getEnvironmentRobots()
   {
      return null;
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
      ContactController contactController = new ContactController();
      contactController.setContactParameters(100000.0, 100.0, 0.5, 0.3);
   }

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {
   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
   }
   
   public static int getNumberOfHatches()
   {
      return HATCHES.size();
   }
   
   public static Hatch getHatch(int hatchNumber)
   {
      return HATCHES.get(hatchNumber);
   }

}
