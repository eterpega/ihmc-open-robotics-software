package us.ihmc.graphicsDescription.yoGraphics;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.GraphicsUpdatable;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.instructions.Graphics3DAddMeshDataInstruction;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * When using this class create and attach a {@code PlaybackListener} to update this object, otherwise the graphics will not be in sync
 */
public class YoGraphicPolygon3D extends YoGraphicAbstractShape implements RemoteYoGraphic, GraphicsUpdatable
{
   private final YoFramePoint[] ccwOrderedPoints;
   private final Point3D[] points;
   private final double height;
   private AppearanceDefinition appearance;
   private final YoInteger numberOfPoints;

   private final Graphics3DObject graphics3dObject;
   private final Graphics3DAddMeshDataInstruction instruction;

   public YoGraphicPolygon3D(String name, int maxNumberOfPolygonVertices, ReferenceFrame referenceFrame, double scale, double height,
                             AppearanceDefinition appearance, YoVariableRegistry registry)
   {
      this(name, maxNumberOfPolygonVertices, new YoFramePoint(name + "FrameOrigin", referenceFrame, registry),
           new YoFrameOrientation(name + "FrameOrientation", referenceFrame, registry), scale, height, appearance, registry);
   }

   public YoGraphicPolygon3D(String name, int maxNumberOfPolygonVertices, YoFramePoint framePoint, YoFrameOrientation frameOrientation, double scale,
                             double height, AppearanceDefinition appearance, YoVariableRegistry registry)
   {
      super(name, framePoint, frameOrientation, scale);
      ccwOrderedPoints = new YoFramePoint[maxNumberOfPolygonVertices];
      points = new Point3D[maxNumberOfPolygonVertices];
      for (int i = 0; i < maxNumberOfPolygonVertices; i++)
      {
         ccwOrderedPoints[i] = new YoFramePoint(name + "Point" + i, framePoint.getReferenceFrame(), registry);
         points[i] = ccwOrderedPoints[i].getFrameTuple().getGeometryObject();
      }
      this.height = height;
      this.appearance = appearance;

      numberOfPoints = new YoInteger(name + "NumberOfPoints", registry);
      graphics3dObject = new Graphics3DObject();
      graphics3dObject.setChangeable(true);
      MeshDataHolder meshDataHolder = MeshDataGenerator.Polygon(points);
      instruction = new Graphics3DAddMeshDataInstruction(meshDataHolder, appearance);
      graphics3dObject.addInstruction(instruction);
   }

   @Override
   public RemoteGraphicType getRemoteGraphicType()
   {
      return RemoteGraphicType.POLYGON_3D;
   }

   @Override
   public YoVariable<?>[] getVariables()
   {
      YoVariable<?>[] yoVariableList = new YoVariable<?>[3 * ccwOrderedPoints.length + 1 + 6];
      for (int i = 0; i < ccwOrderedPoints.length; i++)
      {
         yoVariableList[i * 3] = ccwOrderedPoints[i].getYoX();
         yoVariableList[i * 3 + 1] = ccwOrderedPoints[i].getYoY();
         yoVariableList[i * 3 + 2] = ccwOrderedPoints[i].getYoZ();
      }
      yoVariableList[ccwOrderedPoints.length * 3] = numberOfPoints;
      yoVariableList[ccwOrderedPoints.length * 3 + 1] = yoFramePoint.getYoX();
      yoVariableList[ccwOrderedPoints.length * 3 + 2] = yoFramePoint.getYoY();
      yoVariableList[ccwOrderedPoints.length * 3 + 3] = yoFramePoint.getYoZ();
      yoVariableList[ccwOrderedPoints.length * 3 + 4] = yoFrameOrientation.getRoll();
      yoVariableList[ccwOrderedPoints.length * 3 + 4] = yoFrameOrientation.getPitch();
      yoVariableList[ccwOrderedPoints.length * 3 + 4] = yoFrameOrientation.getYaw();
      return yoVariableList;
   }

   public void setAppearance(AppearanceDefinition appearance)
   {
      this.appearance = appearance;
   }
   
   public synchronized void set(FramePoint3D[] points)
   {
      if (points.length > this.ccwOrderedPoints.length)
         throw new RuntimeException("Cannot plot more vertices than the maximum number");

      int i = 0;
      for (i = 0; i < points.length; i++)
      {
         ReferenceFrame tempFrameReference = points[i].getReferenceFrame();
         points[i].changeFrame(this.yoFramePoint.getReferenceFrame());
         ccwOrderedPoints[i].set(points[i]);
         points[i].changeFrame(tempFrameReference);
      }
      for (; i < ccwOrderedPoints.length; i++)
         ccwOrderedPoints[i].setToNaN();
      numberOfPoints.set(points.length);
      //update();
   }

   private List<Point3DReadOnly> pointList = new ArrayList<>();

   @Override
   protected synchronized void computeRotationTranslation(AffineTransform transform3d)
   {
      //super.computeRotationTranslation(transform3d);
      //update();
   }

   @Override
   public synchronized void update()
   {
      if (numberOfPoints.getIntegerValue() < 3)
      {
         instruction.setMesh(MeshDataGenerator.Cube(0.0, 0.0, 0.0, true));
         return;
      }
      pointList.clear();
      for (int i = 0; i < numberOfPoints.getIntegerValue(); i++)
         pointList.add(ccwOrderedPoints[i].getFrameTuple().getGeometryObject());
      MeshDataHolder meshDataHolder = MeshDataGenerator.Polygon(pointList, numberOfPoints.getIntegerValue());
      instruction.setMesh(meshDataHolder);
      instruction.setAppearance(appearance);
   }

   public synchronized void set(Point3DReadOnly[] points)
   {
      if (points.length > this.ccwOrderedPoints.length)
         throw new RuntimeException("Cannot plot more vertices than the maximum number");
      double x = 0, y = 0, z = 0;
      int i = 0;
      for (i = 0; i < points.length; i++)
      {
         ccwOrderedPoints[i].set(points[i]);
         x += ccwOrderedPoints[i].getX();
         y += ccwOrderedPoints[i].getY();
         z += ccwOrderedPoints[i].getZ();
      }
      x /= points.length;
      y /= points.length;
      z /= points.length;
      yoFramePoint.set(x, y, z);
      for (; i < ccwOrderedPoints.length; i++)
         ccwOrderedPoints[i].setToNaN();
      numberOfPoints.set(points.length);
      //update();
   }

   public synchronized void set(List<Point3DReadOnly> points)
   {
      if (points.size() > this.ccwOrderedPoints.length)
         throw new RuntimeException("Cannot plot more vertices than the maximum number");
      double x = 0, y = 0, z = 0;

      int i = 0;
      for (i = 0; i < points.size(); i++)
      {
         ccwOrderedPoints[i].set(points.get(i));
         x += ccwOrderedPoints[i].getX();
         y += ccwOrderedPoints[i].getY();
         z += ccwOrderedPoints[i].getZ();

      }
      x /= points.size();
      y /= points.size();
      z /= points.size();
      yoFramePoint.set(x, y, z);
      for (; i < ccwOrderedPoints.length; i++)
         ccwOrderedPoints[i].setToNaN();
      numberOfPoints.set(points.size());
      //update();
   }

   public synchronized void setToNaN()
   {
      for (int i = 0; i < ccwOrderedPoints.length; i++)
         ccwOrderedPoints[i].setToNaN();
   }

   @Override
   public double[] getConstants()
   {
      return new double[] {scale, height};
   }

   @Override
   public AppearanceDefinition getAppearance()
   {
      return appearance;
   }

   @Override
   public Graphics3DObject getLinkGraphics()
   {
      return graphics3dObject;
   }
}