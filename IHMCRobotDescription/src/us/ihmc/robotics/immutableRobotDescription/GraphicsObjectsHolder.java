package us.ihmc.robotics.immutableRobotDescription;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;

public interface GraphicsObjectsHolder
{
   Graphics3DObject getCollisionObject(String name);

   Graphics3DObject getGraphicsObject(String name);
}

