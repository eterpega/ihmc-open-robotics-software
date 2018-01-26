package us.ihmc.manipulation.collision;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.simulationconstructionset.physics.CollisionShape;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionShapeFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public abstract class AbstractCollisionShape
{
   protected String name;
   protected SimpleCollisionShapeFactory shapeFactory;
   protected CollisionShape collisionShape;

   /**
    * ReferenceFrame on rigidbody
    */
   protected ReferenceFrame parentFrame;
   /**
    * Customized ReferenceFrame from parentFrame.
    */
   protected ReferenceFrame referenceFrame;
   /**
    * For updating YoGraphics.
    */
   protected YoFramePose yoGraphicFramePose;

   protected YoGraphic yoGraphic;
   protected YoVariableRegistry parentRegistry;

   /**
    * CollisionShapeDescription<?> should be added as a shape on CollisionShape of this class.
    * ex) shapeFactory.addShape(collisionShapeDescription);
    */
   public abstract void createCollisionShape();

   public abstract void createYoGraphic();

   public AbstractCollisionShape(String name, YoVariableRegistry parentRegistry, SimpleCollisionShapeFactory shapeFactory, ReferenceFrame parentFrame,
                                 RigidBodyTransform transformToParentFrame)
   {
      this.name = name;
      this.shapeFactory = shapeFactory;
      this.parentFrame = parentFrame;
      this.parentRegistry = parentRegistry;

      this.referenceFrame = new ReferenceFrame(name + "ReferenceFrame", parentFrame)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(transformToParentFrame);
         }
      };

      this.yoGraphicFramePose = new YoFramePose(name + "pose", ReferenceFrame.getWorldFrame(), parentRegistry);
   }

   public void initialize()
   {
      updateReferenceFrame();

      createCollisionShape();
      updateCollisionShape();
      createYoGraphic();
      updateYoGraphic();
   }

   public void update()
   {
      updateReferenceFrame();
      updateCollisionShape();
      updateYoGraphic();
   }

   /**
    * if collisionshape need any other YoFramePose, this method should be override along with the others. 
    */
   public void updateReferenceFrame()
   {
      referenceFrame.update();
      
      FramePose3D framePose = new FramePose3D(referenceFrame);

      framePose.setToZero();
      framePose.changeFrame(ReferenceFrame.getWorldFrame());

      yoGraphicFramePose.set(framePose);
   }

   public void updateCollisionShape()
   {      
      RigidBodyTransform rigidbodyTransform = new RigidBodyTransform(referenceFrame.getTransformToWorldFrame());
      collisionShape.setTransformToWorld(rigidbodyTransform);
   }

   public void updateYoGraphic()
   {
      yoGraphic.update();
   }

   public void hide()
   {
      yoGraphic.hideGraphicObject();
   }

   public String getName()
   {
      return name;
   }

   public CollisionShape getCollisionShape()
   {
      return collisionShape;
   }

   public void addYoGraphic(YoGraphicsList yoGraphicsList)
   {
      yoGraphicsList.add(yoGraphic);
   }
}
