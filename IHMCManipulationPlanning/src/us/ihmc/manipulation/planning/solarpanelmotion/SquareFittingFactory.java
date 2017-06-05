package us.ihmc.manipulation.planning.solarpanelmotion;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.transformables.Pose;

public class SquareFittingFactory
{
   private PlanarRegion planarRegion;
   private ArrayList<Point3D32> vertices = new ArrayList<Point3D32>();
   private SolarPanel solarPanel;
   
   private Vector3D normalVector;
   private RotationMatrix squareRotationMatrix;
   private Point3D squarePosition;
   
   private double squareSizeX;
   private double squareSizeY;
   
   private LineEquation lineOneAxisX = new LineEquation();
   private LineEquation lineTwoAxisX = new LineEquation();
   
   private LineEquation lineOneAxisY = new LineEquation();
   private LineEquation lineTwoAxisY = new LineEquation();
   
   class LineEquation
   {
      Point3D point;
      Vector3D vector;
      
      LineEquation()
      {
         this.point = new Point3D();
         this.vector = new Vector3D();
      }
      
      LineEquation(Tuple3DBasics point, Vector3D vector)
      {
         this.point = new Point3D(point);
         this.vector = vector;
      }
      
      void setPoint(Tuple3DBasics point)
      {
         this.point = new Point3D(point);
      }
      
      void setVector(Vector3D vector)
      {
         this.vector = vector;
      }
      
      Point3D getPointOnLine(double t)
      {
         return new Point3D(point.getX() + t*vector.getX(), point.getY() + t*vector.getY(), point.getZ() + t*vector.getZ());
      }
      
      Point3D getIntersectedPoint(LineEquation otherLine)
      {
         double x1 = this.point.getX();
         double y1 = this.point.getY();
         double z1 = this.point.getZ();
         double a1 = this.vector.getX();
         double b1 = this.vector.getY();
         double c1 = this.vector.getZ();
         
         double x2 = otherLine.point.getX();
         double y2 = otherLine.point.getY();
         double z2 = otherLine.point.getZ();
         double a2 = otherLine.vector.getX();
         double b2 = otherLine.vector.getY();
         double c2 = otherLine.vector.getZ();
         
         double tOfOtherLine = (a1*y2-a1*y1-b1*x2+b1*x1)/(a2*b1 - a1*b2);
         
         return otherLine.getPointOnLine(tOfOtherLine);
      }
   }   
   
   public SquareFittingFactory()
   {
      
   }
   
   public SquareFittingFactory(PlanarRegion planarRegion)
   {
      this.planarRegion = planarRegion;      
      updateSquare();
   }
   
   public void setPlanarRegion(PlanarRegion planarRegion)
   {
      this.planarRegion = planarRegion;
      updateSquare();
   }
   
   private void updateSquare()
   {
      updateVertices();
            
      updateNormalVector();
      
      fittingSquare();
      
      updateLineEquationsOfSquare();
      
      updateCenterPosition();
      /*
       * here is for converting as we want.
       */      
      updateSolarPanel();  
      
   }
   
   private void updateVertices()
   {
      vertices.clear();
      
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      planarRegion.getTransformToWorld(transformToWorld);
      
      for (int polygonIndex = 0; polygonIndex < planarRegion.getNumberOfConvexPolygons(); polygonIndex++)
      {
         ConvexPolygon2D convexPolygon = planarRegion.getConvexPolygon(polygonIndex);
         MeshDataHolder polygon = MeshDataGenerator.Polygon(transformToWorld, convexPolygon);
         
         PrintTools.info("polygonIndex "+polygonIndex);
         if(polygon != null)
         {
            PrintTools.info("Vectices "+polygon.getVertices().length);
            for(int i=0;i<polygon.getVertices().length;i++)
            {
               PrintTools.info(""+i+" "+polygon.getVertices()[i].getX()+" "+polygon.getVertices()[i].getY()+" "+polygon.getVertices()[i].getZ());
               vertices.add(polygon.getVertices()[i]);
            }
         }         
      }
      
      PrintTools.info("Number Of points are "+ vertices.size());
   }
   

   private void updateNormalVector()
   {      
      this.normalVector = new Vector3D();
      this.planarRegion.getNormal(normalVector); 
   }
         
   private void fittingSquare()
   {
      double appendingPitchDirection = Math.acos(normalVector.getZ());
      if(normalVector.getX() < 0)
         appendingPitchDirection = -appendingPitchDirection;
      double appendingYawDirection = Math.asin(normalVector.getY()/Math.sin(appendingPitchDirection));
      
      PrintTools.info("Currently appendingYawDirection  " + appendingYawDirection);
      PrintTools.info("Currently appendingPitchDirection " + appendingPitchDirection);
           
      int numberOfSampling = 20;
      double minRangeOfSampling = Math.PI * (-44.0/180.0);
      double maxRangeOfSampling = Math.PI * (44.0/180.0);
      double minArea = Double.MAX_VALUE;
      for(int i=0;i<numberOfSampling;i++)
      {
         double appendingYawAngle = (maxRangeOfSampling - minRangeOfSampling) * i/(numberOfSampling-1) + minRangeOfSampling;
         PrintTools.info(""+i+" "+ appendingYawAngle);
         
         RotationMatrix perturbedRotationMatrix = new RotationMatrix();
         perturbedRotationMatrix.appendYawRotation(appendingYawDirection);
         perturbedRotationMatrix.appendPitchRotation(appendingPitchDirection);
         perturbedRotationMatrix.appendYawRotation(appendingYawAngle);
         
         if(minArea > getAreaUnderRotationMatrix(perturbedRotationMatrix))
         {
            minArea = getAreaUnderRotationMatrix(perturbedRotationMatrix);
            squareRotationMatrix = perturbedRotationMatrix;
            PrintTools.info("appendingYawAngle "+appendingYawAngle);
         }
      }      
      
      Vector3D principalAxisX = getPrincipalAxisX(squareRotationMatrix);
      Vector3D principalAxisY = getPrincipalAxisY(squareRotationMatrix);
      
      squareSizeX = getSizeAlongAxis(principalAxisX);
      squareSizeY = getSizeAlongAxis(principalAxisY);
   }
   
   private void updateLineEquationsOfSquare()
   {
      Vector3D principalAxisX = getPrincipalAxisX(squareRotationMatrix);
      Vector3D principalAxisY = getPrincipalAxisY(squareRotationMatrix);
      
      double distance;
      
      distance = 0.0;      
      for(int i=0;i<vertices.size();i++)
      {
         if(distance < getSingedDistancePointAndPlane(vertices.get(i), new Point3D(), principalAxisX))
         {
            distance = getSingedDistancePointAndPlane(vertices.get(i), new Point3D(), principalAxisX);
            lineOneAxisX = new LineEquation(vertices.get(i), principalAxisX);
         }
      }
      
      distance = 0.0;      
      for(int i=0;i<vertices.size();i++)
      {
         if(distance < getSingedDistancePointAndPlane(vertices.get(i), new Point3D(), principalAxisY))
         {
            distance = getSingedDistancePointAndPlane(vertices.get(i), new Point3D(), principalAxisY);
            lineOneAxisY = new LineEquation(vertices.get(i), principalAxisY);
         }
      }
      
      distance = Double.MAX_VALUE;      
      for(int i=0;i<vertices.size();i++)
      {
         if(distance > getSingedDistancePointAndPlane(vertices.get(i), new Point3D(), principalAxisX))
         {
            distance = getSingedDistancePointAndPlane(vertices.get(i), new Point3D(), principalAxisX);
            lineTwoAxisX = new LineEquation(vertices.get(i), principalAxisX);
         }
      }
      
      distance = Double.MAX_VALUE;      
      for(int i=0;i<vertices.size();i++)
      {
         if(distance > getSingedDistancePointAndPlane(vertices.get(i), new Point3D(), principalAxisY))
         {
            distance = getSingedDistancePointAndPlane(vertices.get(i), new Point3D(), principalAxisY);
            lineTwoAxisY = new LineEquation(vertices.get(i), principalAxisY);
         }
      }
   }
   
   private void updateCenterPosition()
   {
      Point3D pointOne = lineOneAxisX.getIntersectedPoint(lineOneAxisY);
      Point3D pointTwo = lineTwoAxisX.getIntersectedPoint(lineTwoAxisY);
      
      Point3D pointThree = lineOneAxisX.getIntersectedPoint(lineTwoAxisY);
      Point3D pointFour  = lineTwoAxisX.getIntersectedPoint(lineOneAxisY);
      
      Point3D centerOne = new Point3D((pointOne.getX()+pointTwo.getX())/2, (pointOne.getY()+pointTwo.getY())/2, (pointOne.getZ()+pointTwo.getZ())/2);
      Point3D centerTwo = new Point3D((pointThree.getX()+pointFour.getX())/2, (pointThree.getY()+pointFour.getY())/2, (pointThree.getZ()+pointFour.getZ())/2);
      
      squarePosition = centerOne;
   }
   
   private void updateSolarPanel()
   {
      Quaternion solarPanelOrientation = new Quaternion(squareRotationMatrix);
      Pose solarPanelPose = new Pose(squarePosition, solarPanelOrientation);
      solarPanel = new SolarPanel(solarPanelPose, squareSizeX, squareSizeY);
   }
   
   
   
   
   
   
   
   
   
   private Vector3D getPrincipalAxisX(RotationMatrix rotationMatrix)
   {
      return new Vector3D(rotationMatrix.getM00(), rotationMatrix.getM10(), rotationMatrix.getM20());
   }
   
   private Vector3D getPrincipalAxisY(RotationMatrix rotationMatrix)
   {
      return new Vector3D(rotationMatrix.getM01(), rotationMatrix.getM11(), rotationMatrix.getM21());
   }
   
   private double getAreaUnderRotationMatrix(RotationMatrix rotationMatrix)
   {
      Vector3D principalAxisX = getPrincipalAxisX(rotationMatrix);
      Vector3D principalAxisY = getPrincipalAxisY(rotationMatrix);
            
      return getSizeAlongAxis(principalAxisX)*getSizeAlongAxis(principalAxisY);
   }

   private double getSizeAlongAxis(Vector3D axis)
   {  
      return Math.abs(getMaximumDistanceFromPlane(new Point3D(), axis) - getMinimumDistanceFromPlane(new Point3D(), axis));
   }
   
   private double getMaximumDistanceFromPlane(Point3D planeCenter, Vector3D planeNormal)
   {
      double distance = 0.0;
      
      for(int i=0;i<vertices.size();i++)
      {
         if(distance < getSingedDistancePointAndPlane(vertices.get(i), planeCenter, planeNormal))
         {
            distance = getSingedDistancePointAndPlane(vertices.get(i), planeCenter, planeNormal);
         }
      }
      
      return distance;
   }
   
   private double getMinimumDistanceFromPlane(Point3D planeCenter, Vector3D planeNormal)
   {
      double distance = Double.MAX_VALUE;
      
      for(int i=0;i<vertices.size();i++)
      {
         if(distance > getSingedDistancePointAndPlane(vertices.get(i), planeCenter, planeNormal))
         {
            distance = getSingedDistancePointAndPlane(vertices.get(i), planeCenter, planeNormal);
         }
      }
      
      return distance;
   }
   
   private double getSingedDistancePointAndPlane(Tuple3DBasics point3d32, Point3D planeCenter, Vector3D planeNormal)
   {
      double planeA = planeNormal.getX();
      double planeB = planeNormal.getY();
      double planeC = planeNormal.getZ();
      double planeD = -planeA*planeCenter.getX()-planeB*planeCenter.getY()-planeC*planeCenter.getZ();

      double distance = (planeA*point3d32.getX() + planeB*point3d32.getY() + planeC*point3d32.getZ() + planeD)/(Math.sqrt(planeA*planeA + planeB*planeB + planeC*planeC));
      
      return distance;
   }
         
   public SolarPanel getSolarPanel()
   {
      return solarPanel;
   }
   
   
   
   
   
   
}