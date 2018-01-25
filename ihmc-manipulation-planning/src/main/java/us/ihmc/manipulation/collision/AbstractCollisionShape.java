package us.ihmc.manipulation.collision;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
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

   protected YoFramePose framePose;
   protected ReferenceFrame referenceFrame;
   protected RigidBodyTransform transformToReferenceFrame;

   protected YoGraphic yoGraphic;
   protected YoVariableRegistry parentRegistry;

   /**
    * CollisionShapeDescription<?> should be added as a shape on CollisionShape of this class.
    * ex) shapeFactory.addShape(collisionShapeDescription);
    */
   public abstract void createCollisionShape();

   public abstract void createYoGraphic();
   
   public AbstractCollisionShape(String name, YoVariableRegistry parentRegistry, SimpleCollisionShapeFactory shapeFactory, ReferenceFrame referenceFrame,
                                 RigidBodyTransform transformToReferenceFrame)
   {
      this.name = name;
      this.shapeFactory = shapeFactory;
      this.referenceFrame = referenceFrame;
      this.transformToReferenceFrame = new RigidBodyTransform(transformToReferenceFrame);
      this.parentRegistry = parentRegistry;
      this.framePose = new YoFramePose(name + "pose", ReferenceFrame.getWorldFrame(), parentRegistry);
   }

   public void initialize()
   {
      updatePose();
      
      createCollisionShape();
      updateCollisionShape();
      createYoGraphic();
      updateYoGraphic();
   }
   
   public void update()
   {
      updatePose();
      updateCollisionShape();
      updateYoGraphic();
   }
   
   public void updatePose()
   {
      RigidBodyTransform rigidbodyTransform = referenceFrame.getTransformToWorldFrame();
      rigidbodyTransform.transform(transformToReferenceFrame);
      
      Point3D position = new Point3D(rigidbodyTransform.getTranslationVector());
      Quaternion orientation = new Quaternion(rigidbodyTransform.getRotationMatrix());
      
      framePose.setPosition(position);
      framePose.setOrientation(orientation);
   }

   public void updateCollisionShape()
   {
      RigidBodyTransform rigidbodyTransform = new RigidBodyTransform();
      framePose.getPose(rigidbodyTransform);
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

   public CollisionShape getCollisionShape()
   {
      return collisionShape;
   }

   public void addYoGraphic(YoGraphicsList yoGraphicsList)
   {
      yoGraphicsList.add(yoGraphic);
   }
}
