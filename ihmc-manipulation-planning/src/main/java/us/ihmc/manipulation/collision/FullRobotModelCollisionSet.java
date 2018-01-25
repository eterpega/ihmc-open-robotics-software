package us.ihmc.manipulation.collision;

import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxOutputConverter;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.simulationconstructionset.physics.collision.CollisionDetectionResult;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionDetector;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionShapeFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FullRobotModelCollisionSet
{
   public static boolean DEBUG = false;
   public FullHumanoidRobotModel fullRobotModel;

   public SimpleCollisionDetector collisionDetector = new SimpleCollisionDetector();
   public CollisionDetectionResult collisionDetectionResult = new CollisionDetectionResult();

   public SimpleCollisionShapeFactory shapeFactory;

   public YoGraphicsList yoGraphicsList;
   
   public KinematicsToolboxOutputConverter outputConverter;

   // TODO
   // update configuration with KinematicsToolboxOutputStatus
   // this should be inherited by FullRobotModel of the Valkyrie and Atlas.   
   // For each
   // Head
   // Chest
   // Pelvis
   // Arms
   // Legs

   public CollisionShapeSphere collisionShapeChest;

   public CollisionShapeSphere collisionShapeHead;

   public FullRobotModelCollisionSet(FullHumanoidRobotModelFactory fullRobotModelFactory, FullHumanoidRobotModel fullRobotModel, YoVariableRegistry parentRegistry)
   {
      this.fullRobotModel = fullRobotModel;

      this.shapeFactory = (SimpleCollisionShapeFactory) collisionDetector.getShapeFactory();
      this.yoGraphicsList = new YoGraphicsList("FullRobotModelCollisionSet");

      RigidBodyTransform transformToReferenceFrame = new RigidBodyTransform();
      collisionShapeChest = new CollisionShapeSphere("chest", parentRegistry, shapeFactory, fullRobotModel.getChest().getBodyFixedFrame(),
                                                     transformToReferenceFrame, 0.2);

      RigidBodyTransform transformToReferenceFrame2 = new RigidBodyTransform();
      collisionShapeHead = new CollisionShapeSphere("head", parentRegistry, shapeFactory, fullRobotModel.getHead().getBodyFixedFrame(),
                                                    transformToReferenceFrame2, 0.3);

      collisionShapeChest.initialize();
      collisionShapeHead.initialize();

      collisionShapeChest.addYoGraphic(yoGraphicsList);
      collisionShapeHead.addYoGraphic(yoGraphicsList);

      collisionShapeChest.getCollisionShape().setCollisionMask(0b01);
      collisionShapeChest.getCollisionShape().setCollisionGroup(0b10);

      collisionShapeHead.getCollisionShape().setCollisionMask(0b10);
      collisionShapeHead.getCollisionShape().setCollisionGroup(0b01);
      
      outputConverter = new KinematicsToolboxOutputConverter(fullRobotModelFactory);
   }

   public void update(KinematicsToolboxOutputStatus configuration)
   {
      fullRobotModel.updateFrames();
      
      collisionShapeChest.update();
      
      collisionShapeHead.update();
   }

   public void hideYoGraphics()
   {
      collisionShapeChest.hide();
      
      collisionShapeHead.hide();
   }
}
