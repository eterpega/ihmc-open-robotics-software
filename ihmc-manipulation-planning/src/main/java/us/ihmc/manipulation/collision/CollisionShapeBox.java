package us.ihmc.manipulation.collision;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.simulationconstructionset.physics.collision.simple.SimpleCollisionShapeFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class CollisionShapeBox extends AbstractCollisionShape
{

   public CollisionShapeBox(String name, YoVariableRegistry parentRegistry, SimpleCollisionShapeFactory shapeFactory, ReferenceFrame parentFrame,
                            RigidBodyTransform transformToParentFrame)
   {
      super(name, parentRegistry, shapeFactory, parentFrame, transformToParentFrame);
      // TODO Auto-generated constructor stub
   }

   @Override
   public void createCollisionShape()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void createYoGraphic()
   {
      // TODO Auto-generated method stub
      
   }

}
