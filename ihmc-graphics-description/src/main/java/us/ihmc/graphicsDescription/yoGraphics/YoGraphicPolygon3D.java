package us.ihmc.graphicsDescription.yoGraphics;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.GraphicsUpdatable;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

public class YoGraphicPolygon3D extends YoGraphicAbstractShape implements RemoteYoGraphic, GraphicsUpdatable
{
   private final YoFramePoint[] ccwOrderedPoints;
   private final double height;
   
   private final Graphics3DObject graphics3dObject;
   
   public YoGraphicPolygon3D(String name, int maxNumberOfPolygonVertices, ReferenceFrame referenceFrame, double scale, double height, YoVariableRegistry registry, Appear)
   {
      this(name, maxNumberOfPolygonVertices, new YoFramePoint(name + "FrameOrigin", referenceFrame, registry),
           new YoFrameOrientation(name + "FrameOrientation", referenceFrame, registry), scale, height, registry);
   }

   public YoGraphicPolygon3D(String name, int maxNumberOfPolygonVertices, YoFramePoint framePoint, YoFrameOrientation frameOrientation, double scale, double height,
                             YoVariableRegistry registry)
   {
      super(name, framePoint, frameOrientation, scale);
      ccwOrderedPoints = new YoFramePoint[maxNumberOfPolygonVertices];
      for (int i = 0; i < maxNumberOfPolygonVertices; i++)
         ccwOrderedPoints[i] = new YoFramePoint(name + "Point" + i, framePoint.getReferenceFrame(), registry);
      this.height = height;

      graphics3dObject = new Graphics3DObject();
      graphics3dObject.setChangeable(true);
   }

   @Override
   public RemoteGraphicType getRemoteGraphicType()
   {
      return RemoteGraphicType.POLYGON_3D;
   }

   @Override
   public YoVariable<?>[] getVariables()
   {
      YoVariable<?>[] yoVariableList = new YoVariable<?>[3 * ccwOrderedPoints.length];
      for (int i = 0; i < ccwOrderedPoints.length; i++)
      {
         yoVariableList[i * 3] = ccwOrderedPoints[i].getYoX();
         yoVariableList[i * 3 + 1] = ccwOrderedPoints[i].getYoY();
         yoVariableList[i * 3 + 2] = ccwOrderedPoints[i].getYoZ();
      }
      return yoVariableList;
   }

   @Override
   public double[] getConstants()
   {
      return new double[]{scale, height};
   }
   
   @Override
   public void update()
   {
      
   }

   @Override
   public AppearanceDefinition getAppearance()
   {
      return null;
   }

   @Override
   public Graphics3DObject getLinkGraphics()
   {
      return graphics3dObject;
   }

}
