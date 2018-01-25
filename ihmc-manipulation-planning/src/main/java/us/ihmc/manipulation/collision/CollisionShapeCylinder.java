package us.ihmc.manipulation.collision;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCylinder;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionShapeFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class CollisionShapeCylinder extends AbstractCollisionShape
{
   private double radius;
   private double height;
   
   private final TranslationReferenceFrame customFrame;
 
   private final FramePose3D pose = new FramePose3D();
   
   public CollisionShapeCylinder(String name, YoVariableRegistry parentRegistry, SimpleCollisionShapeFactory shapeFactory, ReferenceFrame referenceFrame,
                                 RigidBodyTransform transformToReferenceFrame, double radius, double height)
   {
      super(name, parentRegistry, shapeFactory, referenceFrame, transformToReferenceFrame);

      this.radius = radius;
      this.height = height;
      
      customFrame = new TranslationReferenceFrame("customFrame", referenceFrame);
      customFrame.updateTranslation(new Vector3D(0.0, 0.1, 0.0));
   }
   
   @Override
   public void createCollisionShape()
   {
      CollisionShapeDescription<?> collisionShapeDescription = shapeFactory.createCylinder(radius, height);
      collisionShape = shapeFactory.addShape(collisionShapeDescription);
   }
   
   @Override
   public void createYoGraphic(Vector3DReadOnly translationToCentroid)
   {  
      customFrame.update();
      pose.setToZero(customFrame);
      pose.changeFrame(ReferenceFrame.getWorldFrame());
      
      
      YoFrameVector cylinderVector = new YoFrameVector(name+"cylinderVector", ReferenceFrame.getWorldFrame(), parentRegistry);
      cylinderVector.set(translationToCentroid);
      
      yoGraphic = new YoGraphicCylinder(name + "yographics", framePose.getPosition(), cylinderVector, YoAppearance.Yellow(), 0.1);
      yoGraphic.setVisible(true);
   }
}
