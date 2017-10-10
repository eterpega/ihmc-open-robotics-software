package us.ihmc.geometry.polytope;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.referenceFrame.FrameGeometryObject;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class FramePolytopeVertex extends FrameGeometryObject<FramePolytopeVertex, PolytopeVertex> implements PolytopeVertexBasics
{
   private final PolytopeVertex polytopeVertex;
   private final FramePoint3D framePosition;
   
   public FramePolytopeVertex(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      this(referenceFrame, new PolytopeVertex(x,y,z));
   }
   
   public FramePolytopeVertex(FramePoint3D vertex)
   {
      this(vertex.getReferenceFrame(), new PolytopeVertex(vertex.getPoint()));
   }

   public FramePolytopeVertex(ReferenceFrame referenceFrame, Point3D vertex)
   {
      this(referenceFrame, new PolytopeVertex(vertex));
   }
   
   public FramePolytopeVertex(ReferenceFrame referenceFrame, PolytopeVertex vertex)
   {
      super(referenceFrame, vertex);
      this.polytopeVertex = vertex;
      this.framePosition = new FramePoint3D(referenceFrame);
   }
   
   public List<? extends PolytopeHalfEdgeBasics> getAssociatedEdges()
   {
      return polytopeVertex.getAssociatedEdges();
   }
   
   public PolytopeHalfEdgeBasics getAssociatedEdge(int index)
   {
      return polytopeVertex.getAssociatedEdge(index);
   }
   
   public void removeAssociatedEdge(FramePolytopeHalfEdge framePolytopeHalfEdge)
   {
      checkReferenceFrameMatch(framePolytopeHalfEdge);
      polytopeVertex.removeAssociatedEdge(framePolytopeHalfEdge.getGeometryObject());
   }
   
   public void clearAssociatedEdgeList()
   {
      polytopeVertex.clearAssociatedEdgeList();
   }

   public void copyEdges(List<FramePolytopeHalfEdge> edgeListToCopy)
   {
      polytopeVertex.clearAssociatedEdgeList();
      for(int i = 0; i < edgeListToCopy.size(); i++)
      {
         checkReferenceFrameMatch(edgeListToCopy.get(i));
         polytopeVertex.addAssociatedEdge(edgeListToCopy.get(i).getGeometryObject());
      }
   }
   
   public void addAssociatedEdge(FramePolytopeHalfEdge framePolytopeHalfEdgeToAdd)
   {
      checkReferenceFrameMatch(framePolytopeHalfEdgeToAdd);
      polytopeVertex.addAssociatedEdge(framePolytopeHalfEdgeToAdd.getGeometryObject());
   }
   
   public boolean isAssociatedWithEdge(FramePolytopeHalfEdge framePolytopeHalfEdgeToCheck)
   {
      checkReferenceFrameMatch(framePolytopeHalfEdgeToCheck);
      return polytopeVertex.isAssociatedWithEdge(framePolytopeHalfEdgeToCheck.getGeometryObject());
   }
   
   public boolean isAssociatedWithEdge(FramePolytopeHalfEdge framePolytopeHalfEdgeToCheck, double epsilon)
   {
      checkReferenceFrameMatch(framePolytopeHalfEdgeToCheck);
      return polytopeVertex.isAssociatedWithEdge(framePolytopeHalfEdgeToCheck.getGeometryObject(), epsilon);
   }
   
   public int getNumberOfAssociatedEdges()
   {
      return polytopeVertex.getNumberOfAssociatedEdges();
   }
   
   public FramePoint3D getPosition()
   {
      framePosition.set(polytopeVertex.getPosition());
      return framePosition;
   }
   
   public double dot(FrameVector3D vector)
   {
      checkReferenceFrameMatch(vector);
      return polytopeVertex.dot(vector.getGeometryObject());
   }
   
   public double dot(Vector3DReadOnly vector)
   {
      return polytopeVertex.dot(vector);
   }
   
   public String toString()
   {
      return polytopeVertex.toString() + " -(" + referenceFrame.getName() + ")";
   }
  
   public double getX()
   {
      return polytopeVertex.getX();
   }
   
   public double getY()
   {
      return polytopeVertex.getY();
   }
   
   public double getZ()
   {
      return polytopeVertex.getZ();
   }
   
   public double getElement(int index)
   {
      return polytopeVertex.getElement(index);
   }
   
   public boolean isAnyFaceMarked()
   {
      return polytopeVertex.isAnyFaceMarked();
   }
   
   @Override
   public void setX(double x)
   {
      polytopeVertex.setX(x);
   }

   @Override
   public void setY(double y)
   {
      polytopeVertex.setY(y);
   }

   @Override
   public void setZ(double z)
   {
      polytopeVertex.setZ(z);
   }

   @Override
   public double getShortestDistanceTo(Point3DReadOnly point)
   {
      // TODO Auto-generated method stub
      return 0;
   }

   @Override
   public void getSupportVectorDirectionTo(Point3DReadOnly point, Vector3D supportVectorToPack)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void getSupportVectorJacobianTo(Point3DReadOnly point, DenseMatrix64F jacobianToPack)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public Simplex getSmallestSimplexMemberReference(Point3DReadOnly point)
   {
      // TODO Auto-generated method stub
      return null;
   }
}
