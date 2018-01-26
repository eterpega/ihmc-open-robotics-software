package us.ihmc.manipulation.collision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxOutputConverter;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.physics.collision.CollisionDetectionResult;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionShapeFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FullRobotModelCollisionSet
{
   private static boolean DEBUG = false;
   private FullHumanoidRobotModel fullRobotModel;

   private SimpleCollisionDetector collisionDetector = new SimpleCollisionDetector();
   private CollisionDetectionResult collisionDetectionResult = new CollisionDetectionResult();

   private SimpleCollisionShapeFactory shapeFactory;

   private YoGraphicsList yoGraphicsList;

   private KinematicsToolboxOutputConverter outputConverter;

   // TODO
   // update configuration with KinematicsToolboxOutputStatus
   // this should be inherited by FullRobotModel of the Valkyrie and Atlas.   
   // For each
   // Head
   // Chest
   // Pelvis
   // Arms
   // Legs

   private final List<AbstractCollisionShape> collisionShapesList;
   private final Map<String, AbstractCollisionShape> nameToCollisionShapeMap;

   public FullRobotModelCollisionSet(FullHumanoidRobotModelFactory fullRobotModelFactory, FullHumanoidRobotModel fullRobotModel,
                                     YoVariableRegistry parentRegistry)
   {
      this.fullRobotModel = fullRobotModel;

      this.shapeFactory = (SimpleCollisionShapeFactory) collisionDetector.getShapeFactory();
      this.yoGraphicsList = new YoGraphicsList("FullRobotModelCollisionSet");

      collisionShapesList = new ArrayList<>();
      nameToCollisionShapeMap = new HashMap<>();

      /*
       * start to add shapes.
       */
      RigidBodyTransform transformToReferenceFrame = new RigidBodyTransform();
//      collisionShapesList.add(new CollisionShapeSphere("chest", parentRegistry, shapeFactory, fullRobotModel.getChest().getBodyFixedFrame(),
//                                                       transformToReferenceFrame, 0.25));
      collisionShapesList.add(new CollisionShapeSphere("head", parentRegistry, shapeFactory, fullRobotModel.getHead().getBodyFixedFrame(),
                                                       transformToReferenceFrame, 0.2));
      
      RigidBodyTransform rightHandOffset = new RigidBodyTransform();
      rightHandOffset.appendTranslation(-0.035, -0.02, 0.0);
      rightHandOffset.appendPitchRotation(Math.PI*0.5);
      collisionShapesList.add(new CollisionShapeCylinder("righthand", parentRegistry, shapeFactory, fullRobotModel.getHand(RobotSide.RIGHT).getBodyFixedFrame(),
                                                       rightHandOffset, 0.10, 0.05));
      
      RigidBodyTransform leftHandOffset = new RigidBodyTransform();
      leftHandOffset.appendTranslation(-0.035, 0.02, 0.0);
      leftHandOffset.appendPitchRotation(Math.PI*0.5);
      collisionShapesList.add(new CollisionShapeCylinder("lefthand", parentRegistry, shapeFactory, fullRobotModel.getHand(RobotSide.LEFT).getBodyFixedFrame(),
                                                         leftHandOffset, 0.10, 0.05));
            

      /*
       * define as a map.
       */
      collisionShapesList.forEach(collisionShape -> nameToCollisionShapeMap.put(collisionShape.getName(), collisionShape));

      for (int i = 0; i < collisionShapesList.size(); i++)
      {
         PrintTools.info("" + i + " " + collisionShapesList.get(i).getName());
         collisionShapesList.get(i).initialize();
         collisionShapesList.get(i).addYoGraphic(yoGraphicsList);
      }

      /*
       * grouping and masking.
       */
      //      collisionShapeChest.getCollisionShape().setCollisionMask(0b01);
      //      collisionShapeChest.getCollisionShape().setCollisionGroup(0b10);
      //
      //      collisionShapeHead.getCollisionShape().setCollisionMask(0b10);
      //      collisionShapeHead.getCollisionShape().setCollisionGroup(0b01);

      outputConverter = new KinematicsToolboxOutputConverter(fullRobotModelFactory);
   }
   
   public void update(KinematicsToolboxOutputStatus configuration)
   {
      fullRobotModel.updateFrames();

      for (int i = 0; i < collisionShapesList.size(); i++)
         collisionShapesList.get(i).update();
   }

   public void hideYoGraphics()
   {
      for (int i = 0; i < collisionShapesList.size(); i++)
         collisionShapesList.get(i).hide();
   }
   
   public void registerYoGraphicsList(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
   }
}
