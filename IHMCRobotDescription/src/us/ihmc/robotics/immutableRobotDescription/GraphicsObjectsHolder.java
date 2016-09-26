package us.ihmc.robotics.immutableRobotDescription;

import us.ihmc.robotics.immutableRobotDescription.graphics.GraphicsGroupDescription;

public interface GraphicsObjectsHolder
{
   GraphicsGroupDescription getCollisionObject(String name);

   GraphicsGroupDescription getGraphicsObject(String name);
}

