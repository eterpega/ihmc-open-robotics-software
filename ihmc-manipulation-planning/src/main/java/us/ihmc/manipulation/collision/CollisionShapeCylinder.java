package us.ihmc.manipulation.collision;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCylinder;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionShapeFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class CollisionShapeCylinder extends AbstractCollisionShape
{
   private double radius;
   private double height;

   private final YoFrameVector yoCylinderVector;

   public CollisionShapeCylinder(String name, YoVariableRegistry parentRegistry, SimpleCollisionShapeFactory shapeFactory, ReferenceFrame referenceFrame,
                                 RigidBodyTransform transformToReferenceFrame, double radius, double height)
   {
      super(name, parentRegistry, shapeFactory, referenceFrame, transformToReferenceFrame);

      this.radius = radius;
      this.height = height;

      this.yoCylinderVector = new YoFrameVector(name + "cylinderVector", ReferenceFrame.getWorldFrame(), parentRegistry);
   }

   @Override
   public void createCollisionShape()
   {
      CollisionShapeDescription<?> collisionShapeDescription = shapeFactory.createCylinder(radius, height);
      collisionShape = shapeFactory.addShape(collisionShapeDescription);
   }

   @Override
   public void createYoGraphic()
   {
      yoGraphic = new YoGraphicCylinder(name + "yographics", yoGraphicFramePose.getPosition(), yoCylinderVector, YoAppearance.Yellow(), radius);
      yoGraphic.setVisible(true);
   }

   @Override
   public void updateReferenceFrame()
   {
      referenceFrame.update();

      FramePose3D framePoseStart = new FramePose3D(referenceFrame);
      framePoseStart.setToZero(referenceFrame);
      framePoseStart.changeFrame(ReferenceFrame.getWorldFrame());

      yoGraphicFramePose.set(framePoseStart);

      Vector3D vectorToReferenceFrame = new Vector3D(0, 0, height);

      FramePose3D framePoseEnd = new FramePose3D(referenceFrame);
      framePoseEnd.setToZero(referenceFrame);
      framePoseEnd.appendTranslation(vectorToReferenceFrame);
      framePoseEnd.changeFrame(ReferenceFrame.getWorldFrame());

      Vector3D vectorToWorld = new Vector3D();
      vectorToWorld.setX(framePoseEnd.getPosition().getX() - framePoseStart.getPosition().getX());
      vectorToWorld.setY(framePoseEnd.getPosition().getY() - framePoseStart.getPosition().getY());
      vectorToWorld.setZ(framePoseEnd.getPosition().getZ() - framePoseStart.getPosition().getZ());

      yoCylinderVector.set(vectorToWorld);
   }
}
