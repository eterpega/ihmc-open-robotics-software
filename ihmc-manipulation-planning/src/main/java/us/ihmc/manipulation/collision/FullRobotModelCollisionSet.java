package us.ihmc.manipulation.collision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxOutputConverter;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
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
      // Head, Chest and Pelvis
      RigidBodyTransform transformToReferenceFrame = new RigidBodyTransform();
      //      collisionShapesList.add(new CollisionShapeSphere("chest", parentRegistry, shapeFactory, fullRobotModel.getChest().getBodyFixedFrame(),
      //                                                       transformToReferenceFrame, 0.25));
//      collisionShapesList.add(new CollisionShapeSphere("head", parentRegistry, shapeFactory, fullRobotModel.getHead().getBodyFixedFrame(),
//                                                       transformToReferenceFrame, 0.2));

      // Hands
//      RigidBodyTransform rightHandOffset = new RigidBodyTransform();
//      rightHandOffset.appendTranslation(-0.035, -0.02, 0.0);
//      rightHandOffset.appendPitchRotation(Math.PI * 0.5);
//      
//      collisionShapesList.add(new CollisionShapeCylinder("righthand", parentRegistry, shapeFactory, fullRobotModel.getHandControlFrame(RobotSide.RIGHT),
//                                                         rightHandOffset, 0.10, 0.05));
//
//      RigidBodyTransform leftHandOffset = new RigidBodyTransform();
//      leftHandOffset.appendTranslation(-0.035, 0.02, 0.0);
//      leftHandOffset.appendPitchRotation(Math.PI * 0.5);
//      collisionShapesList.add(new CollisionShapeCylinder("lefthand", parentRegistry, shapeFactory, fullRobotModel.getHand(RobotSide.LEFT).getBodyFixedFrame(),
//                                                         leftHandOffset, 0.10, 0.05));

      // Foots

      // Arms
      InverseDynamicsJoint rightElbowJoint = fullRobotModel.getHand(RobotSide.RIGHT).getParentJoint().getPredecessor().getParentJoint().getPredecessor()
                                                           .getParentJoint().getPredecessor().getParentJoint();
      // second joint of the shoulder
      InverseDynamicsJoint rightShoulderJoint = rightElbowJoint.getPredecessor().getParentJoint().getPredecessor().getParentJoint();
      InverseDynamicsJoint leftElbowJoint = fullRobotModel.getHand(RobotSide.LEFT).getParentJoint().getPredecessor().getParentJoint().getPredecessor()
                                                          .getParentJoint().getPredecessor().getParentJoint();

      FramePose3D framePoseWrist = new FramePose3D(fullRobotModel.getHand(RobotSide.RIGHT).getParentJoint().getPredecessor().getBodyFixedFrame());
      FramePose3D framePoseElbow = new FramePose3D(rightElbowJoint.getFrameBeforeJoint());
      FramePose3D framePoseShoulder = new FramePose3D(rightShoulderJoint.getFrameBeforeJoint());

      framePoseWrist.changeFrame(ReferenceFrame.getWorldFrame());
      framePoseElbow.changeFrame(ReferenceFrame.getWorldFrame());
      framePoseShoulder.changeFrame(ReferenceFrame.getWorldFrame());
      double distanceFromWristToElbow = framePoseWrist.getPositionDistance(framePoseElbow);
      double distanceFromElbowToShoulder = framePoseElbow.getPositionDistance(framePoseShoulder);
      
      RigidBodyTransform armOffsetOne = new RigidBodyTransform();
      armOffsetOne.appendRollRotation(-Math.PI*0.5);
      RigidBodyTransform armOffsetTwo = new RigidBodyTransform();
      armOffsetTwo.appendRollRotation(Math.PI*0.5);

      collisionShapesList.add(new CollisionShapeCylinder("rightlowerarm", parentRegistry, shapeFactory, rightElbowJoint.getFrameAfterJoint(), armOffsetTwo,
                                                         0.08, distanceFromWristToElbow));
      collisionShapesList.add(new CollisionShapeCylinder("rightUpperarm", parentRegistry, shapeFactory, rightElbowJoint.getFrameBeforeJoint(), armOffsetOne,
                                                         0.08, distanceFromElbowToShoulder));
      collisionShapesList.add(new CollisionShapeCylinder("leftlowerarm", parentRegistry, shapeFactory, leftElbowJoint.getFrameAfterJoint(), armOffsetOne,
                                                         0.08, distanceFromWristToElbow));
      collisionShapesList.add(new CollisionShapeCylinder("leftUpperarm", parentRegistry, shapeFactory, leftElbowJoint.getFrameBeforeJoint(), armOffsetTwo,
                                                         0.08, distanceFromElbowToShoulder));

      // Legs
      InverseDynamicsJoint rightKneeJoint = fullRobotModel.getFoot(RobotSide.RIGHT).getParentJoint().getPredecessor().getParentJoint().getPredecessor()
                                                          .getParentJoint();
      InverseDynamicsJoint rightPelvisJoint = rightKneeJoint.getPredecessor().getParentJoint().getPredecessor().getParentJoint().getPredecessor()
                                                            .getParentJoint();
      InverseDynamicsJoint leftKneeJoint = fullRobotModel.getFoot(RobotSide.LEFT).getParentJoint().getPredecessor().getParentJoint().getPredecessor()
                                                         .getParentJoint();

      FramePose3D framePoseAnkle = new FramePose3D(fullRobotModel.getFoot(RobotSide.RIGHT).getParentJoint().getPredecessor().getBodyFixedFrame());
      FramePose3D framePoseKnee = new FramePose3D(rightKneeJoint.getFrameBeforeJoint());
      FramePose3D framePosePelvis = new FramePose3D(rightPelvisJoint.getFrameBeforeJoint());

      framePoseAnkle.changeFrame(ReferenceFrame.getWorldFrame());
      framePoseKnee.changeFrame(ReferenceFrame.getWorldFrame());
      framePosePelvis.changeFrame(ReferenceFrame.getWorldFrame());
      double distanceFromAnkleToKnee = framePoseAnkle.getPositionDistance(framePoseKnee);
      double distanceFromKneeToPelvis = framePoseKnee.getPositionDistance(framePosePelvis);

      RigidBodyTransform lowerLegOffset = new RigidBodyTransform();
      lowerLegOffset.appendTranslation(-0.00, -0.00, -distanceFromAnkleToKnee);
      collisionShapesList.add(new CollisionShapeCylinder("rightlowerleg", parentRegistry, shapeFactory, rightKneeJoint.getFrameAfterJoint(), lowerLegOffset,
                                                         0.08, distanceFromAnkleToKnee));
      collisionShapesList.add(new CollisionShapeCylinder("rightupperleg", parentRegistry, shapeFactory, rightKneeJoint.getFrameBeforeJoint(),
                                                         new RigidBodyTransform(), 0.08, distanceFromKneeToPelvis));
      collisionShapesList.add(new CollisionShapeCylinder("leftlowerleg", parentRegistry, shapeFactory, leftKneeJoint.getFrameAfterJoint(), lowerLegOffset, 0.08,
                                                         distanceFromAnkleToKnee));
      collisionShapesList.add(new CollisionShapeCylinder("leftupperleg", parentRegistry, shapeFactory, leftKneeJoint.getFrameBeforeJoint(),
                                                         new RigidBodyTransform(), 0.08, distanceFromKneeToPelvis));

      /*
       * define on map.
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
