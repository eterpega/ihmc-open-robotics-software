package us.ihmc.simulationConstructionSetTools.util.environments;

import java.util.List;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationConstructionSetTools.robotController.ContactController;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.BoxTerrainObject;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;


public class HatchEnvironment implements CommonAvatarEnvironmentInterface
{
   private final CombinedTerrainObject3D combinedTerrainObject;
      
   private static final double HATCH_OPENING_WIDTH = .86 - 0.02; // 0.84
   private static final double HATCH_UPENING_HEIGHT = 1.7 - 0.15; // 1.55
   private static final double HATCH_UPENING_HEIGHT_OFF_GROUND = 0.20; //0.15; 20 w/ 0.5
   private static final double FORWARD_OFFSET = 0.21; //0.22; //0.21f; //2f; //2 + 0.21f; //1f;
   private static final double SIDEWAY_OFFSET = 0.08; //0.5f; //1 + 0.08f;
   private static final double HATCH_THICKNESS = 0.05; //0.15; (maybe max at 0.13)
   
   private static final int NUMBER_OF_HATCHES = 4;                               // Simulation values:
   private static final double FORWARD_OFFSETS[] = {1.21, 3.71, 6.21, 8.71};     // {1.21, 3.71, 6.21, 8.71};
   private static final double SIDEWAY_OFFSETS[] = {0.08, 1.08, 0.08, 0.08};     // {0.08, 1.08, 0.08, 0.08};
   private static final double STEP_HEIGHTS[] = {0.15, 0.05, 0.20, 0.10};        // {0.15, 0.05, 0.20, 0.10};
   private static final double OPENING_HEIGHTS[] = {1.60, 1.60, 1.60, 1.60};     // {1.55, 1.55, 1.55, 1.55};
   private static final double OPENING_WIDTHS[] = {0.86, 0.86, 0.86, 0.86};      // {0.86, 0.86, 0.86, 0.86};
   private static final double OPENING_THICKNESSES[] = {0.10, 0.10, 0.03, 0.10}; // {0.12, 0.12, 0.05, 0.12};
   
   public HatchEnvironment()
   {
      combinedTerrainObject = new CombinedTerrainObject3D(getClass().getSimpleName());
      combinedTerrainObject.addTerrainObject(setUpGround("Ground"));
      combinedTerrainObject.addTerrainObject(addHatch("IBims1Hatch", FORWARD_OFFSETS[0], SIDEWAY_OFFSETS[0], STEP_HEIGHTS[0], OPENING_HEIGHTS[0], OPENING_WIDTHS[0], OPENING_THICKNESSES[0]));
      combinedTerrainObject.addTerrainObject(addHatch("IBims2Hatch", FORWARD_OFFSETS[1], SIDEWAY_OFFSETS[1], STEP_HEIGHTS[1], OPENING_HEIGHTS[1], OPENING_WIDTHS[1], OPENING_THICKNESSES[1]));
      combinedTerrainObject.addTerrainObject(addHatch("IBims3Hatch", FORWARD_OFFSETS[2], SIDEWAY_OFFSETS[2], STEP_HEIGHTS[2], OPENING_HEIGHTS[2], OPENING_WIDTHS[2], OPENING_THICKNESSES[2]));
      combinedTerrainObject.addTerrainObject(addHatch("IBims4Hatch", FORWARD_OFFSETS[3], SIDEWAY_OFFSETS[3], STEP_HEIGHTS[3], OPENING_HEIGHTS[3], OPENING_WIDTHS[3], OPENING_THICKNESSES[3]));
   }
   
   private CombinedTerrainObject3D setUpGround(String name)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);
      combinedTerrainObject.addBox(-10.0, -10.0, 10.0, 10.0, -0.05, 0.0, YoAppearance.Gray());

      return combinedTerrainObject;
   }
   
   private CombinedTerrainObject3D addHatch(String name, double forwardOffset, double sidewayOffset, double stepHeight, double openingHeight, double openingWidth, double openingThickness)
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D(name);

      combinedTerrainObject.addBox(forwardOffset, sidewayOffset+(openingWidth/2.0)+0.5,  forwardOffset+openingThickness, sidewayOffset-(openingWidth/2.0)-0.5, 0, stepHeight, YoAppearance.DarkGray());

      combinedTerrainObject.addBox(forwardOffset,sidewayOffset+openingWidth/2.0,  forwardOffset+openingThickness,sidewayOffset+(openingWidth/2.0)+0.5, 0, openingHeight+stepHeight, YoAppearance.DarkGray());
      combinedTerrainObject.addBox(forwardOffset,sidewayOffset-openingWidth/2.0,  forwardOffset+openingThickness,sidewayOffset-(openingWidth/2.0)-0.5, 0, openingHeight+stepHeight, YoAppearance.DarkGray());

      BoxTerrainObject hatchTop = new BoxTerrainObject(forwardOffset,sidewayOffset+(openingWidth/2)+0.5,  forwardOffset+openingThickness,sidewayOffset-(openingWidth/2.0)-0.5, openingHeight+stepHeight, 1.95, YoAppearance.DarkGray());
      combinedTerrainObject.addStaticLinkGraphics(hatchTop.getLinkGraphics());
      
      return combinedTerrainObject;
   }
   
   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return combinedTerrainObject;
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
      return NUMBER_OF_HATCHES;
   }
   
   public static Point3D getHatchFrameOffset(int hatch)
   {
      return new Point3D(FORWARD_OFFSETS[hatch], SIDEWAY_OFFSETS[hatch], 0.0);
   }
   
   public static double getHatchThickness(int hatch)
   {
      return OPENING_THICKNESSES[hatch];
   }
   
   public static double getHatchLowerHeight(int hatch)
   {
      return STEP_HEIGHTS[hatch];
   }
   
   public static double getHatchUpperHeight(int hatch)
   {
      return OPENING_HEIGHTS[hatch];
   }
   
   public static double getHatchWidth(int hatch)
   {
      return OPENING_WIDTHS[hatch];
   }

}
