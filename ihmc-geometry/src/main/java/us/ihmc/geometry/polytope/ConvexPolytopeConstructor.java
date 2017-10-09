package us.ihmc.geometry.polytope;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class ConvexPolytopeConstructor
{
   private static final double EPSILON = Epsilons.ONE_TEN_THOUSANDTH;

   public static ConvexPolytope constructUnitCube()
   {
      ConvexPolytope polytope = new ConvexPolytope();
      polytope.addVertex(new PolytopeVertex(0.0, 0.0, 0.0), EPSILON);
      polytope.addVertex(new PolytopeVertex(1.0, 0.0, 0.0), EPSILON);
      polytope.addVertex(new PolytopeVertex(0.0, 1.0, 0.0), EPSILON);
      polytope.addVertex(new PolytopeVertex(1.0, 1.0, 0.0), EPSILON);
      polytope.addVertex(new PolytopeVertex(0.0, 0.0, 1.0), EPSILON);
      polytope.addVertex(new PolytopeVertex(1.0, 0.0, 1.0), EPSILON);
      polytope.addVertex(new PolytopeVertex(0.0, 1.0, 1.0), EPSILON);
      polytope.addVertex(new PolytopeVertex(1.0, 1.0, 1.0), EPSILON);
      return polytope;
   }

   public static ConvexPolytope constructCube(double sideLength, Point3D center)
   {
      ConvexPolytope polytope = new ConvexPolytope();

      return polytope;
   }

   public static ConvexPolytope constructCuboid(double lengthX, double lengthY, double lengthZ)
   {
      ConvexPolytope polytope = new ConvexPolytope();

      return polytope;
   }

   /**
    * Constructs a icosahedron that envelops the sphere to be created
    * @param radius
    * @param center
    * @param edgeLengthForDiscretization
    * @return
    */
   public static ConvexPolytope constructSphere(double radius, Point3D center, int recursionLevel)
   {
      ConvexPolytope polytope = new ConvexPolytope();
      int t1 = 1;
      double t2 = 1 + Math.sqrt(5) / 2;
      double scale = radius / Math.sqrt(t1 * t1 + t2 * t2);
      t1 *= scale;
      t2 *= scale;
      
      polytope.addVertex(center.getX() - t1, center.getY() + t2, center.getZ(), EPSILON);
      polytope.addVertex(center.getX() + t1, center.getY() + t2, center.getZ(), EPSILON);
      polytope.addVertex(center.getX() - t1, center.getY() - t2, center.getZ(), EPSILON);
      polytope.addVertex(center.getX() + t1, center.getY() - t2, center.getZ(), EPSILON);

      polytope.addVertex(center.getX(), center.getY() - t1, center.getZ() + t2, EPSILON);
      polytope.addVertex(center.getX(), center.getY() + t1, center.getZ() + t2, EPSILON);
      polytope.addVertex(center.getX(), center.getY() - t1, center.getZ() - t2, EPSILON);
      polytope.addVertex(center.getX(), center.getY() + t1, center.getZ() - t2, EPSILON);

      polytope.addVertex(center.getX() + t2, center.getY(), center.getZ() - t1, EPSILON);
      polytope.addVertex(center.getX() + t2, center.getY(), center.getZ() + t1, EPSILON);
      polytope.addVertex(center.getX() - t2, center.getY(), center.getZ() - t1, EPSILON);
      polytope.addVertex(center.getX() - t2, center.getY(), center.getZ() + t1, EPSILON);

      List<PolytopeHalfEdge> edges = new ArrayList<>((int)(120 * Math.pow(4, recursionLevel)));
      for (int i = 0; i < recursionLevel; i++)
      {
         edges.clear();
         edges.addAll(polytope.getEdges());
         scale = radius / Math.sqrt( radius * radius - edges.get(0).getEdgeVector().dot(edges.get(0).getEdgeVector()) ) / 2.0;
         for(int j = 0; j < edges.size(); )
         {
            PolytopeVertex origin = edges.get(j).getOriginVertex();
            PolytopeVertex destination = edges.get(j).getDestinationVertex();
            PolytopeVertex newVertex = new PolytopeVertex((origin.getX() + destination.getX()) * scale, (origin.getY() + destination.getY()) * scale, (origin.getZ() + destination.getZ()) * scale);
            polytope.addVertex(newVertex, EPSILON);
            edges.remove(edges.get(j));
            edges.remove(edges.get(j).getTwinHalfEdge());
         }
      }

      return polytope;
   }

   public static ConvexPolytope constructUnitSphere(int recursionLevel)
   {
      return constructSphere(1.0, new Point3D(), recursionLevel);
   }

   /**
    * Creates a polytope by discretizing the curved surface of the cylinder
    * @param center location of the polytope centroid
    * @param radius radius of the cylinder to be made
    * @param length lenght of the cylinder to be made
    * @param numberOfDivisionsForCurvedSurface
    * @return
    */
   public static ConvexPolytope constructCylinder(Point3D center, double radius, double length, int numberOfDivisionsForCurvedSurface)
   {
      ConvexPolytope polytope = new ConvexPolytope();
      double vertexAngle = 2 * Math.PI / numberOfDivisionsForCurvedSurface;
      double enclosingRadius = radius / Math.cos(vertexAngle / 2.0);
      for (int i = 0; i < numberOfDivisionsForCurvedSurface; i++)
         polytope.addVertex(new PolytopeVertex(center.getX() + enclosingRadius * Math.cos(i * vertexAngle), center.getY() + Math.sin(i * vertexAngle),
                                               center.getZ() - length / 2.0),
                            EPSILON);
      for (int i = 0; i < numberOfDivisionsForCurvedSurface; i++)
         polytope.addVertex(new PolytopeVertex(center.getX() + enclosingRadius * Math.cos(i * vertexAngle), center.getY() + Math.sin(i * vertexAngle),
                                               center.getZ() + length / 2.0),
                            EPSILON);
      return polytope;
   }

   public static FrameConvexPolytope constructUnitCube(ReferenceFrame frame)
   {
      FrameConvexPolytope polytope = new FrameConvexPolytope(frame, constructUnitCube());
      return polytope;
   }

   public static ConvexPolytope constructBoxWithCenterAtZero(double halfLengthX, double halfWidthY, double halfHeightZ)
   {
      ConvexPolytope polytope = new ConvexPolytope();

      polytope.addVertex(new PolytopeVertex(-halfLengthX, -halfWidthY, -halfHeightZ), EPSILON);
      polytope.addVertex(new PolytopeVertex(halfLengthX, -halfWidthY, -halfHeightZ), EPSILON);
      polytope.addVertex(new PolytopeVertex(halfLengthX, halfWidthY, -halfHeightZ), EPSILON);
      polytope.addVertex(new PolytopeVertex(-halfLengthX, halfWidthY, -halfHeightZ), EPSILON);
      polytope.addVertex(new PolytopeVertex(-halfLengthX, -halfWidthY, halfHeightZ), EPSILON);
      polytope.addVertex(new PolytopeVertex(halfLengthX, -halfWidthY, halfHeightZ), EPSILON);
      polytope.addVertex(new PolytopeVertex(halfLengthX, halfWidthY, halfHeightZ), EPSILON);
      polytope.addVertex(new PolytopeVertex(-halfLengthX, halfWidthY, halfHeightZ), EPSILON);

      return polytope;
   }

   public static ConvexPolytope constructRandomSphereOutlinedPolytope(Random random, int numberOfPoints, double radius, double xyzBoundary)
   {
      ConvexPolytope polytope = new ConvexPolytope();

      Point3D sphereCenter = EuclidCoreRandomTools.nextPoint3D(random, xyzBoundary, xyzBoundary, xyzBoundary);
      for (int i = 0; i < numberOfPoints; i++)
      {
         Vector3D randomVector = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, radius);
         Point3D point = new Point3D(sphereCenter);
         point.add(randomVector);
         polytope.addVertex(point, EPSILON);
      }

      return polytope;
   }

   public static ConvexPolytope constructSinglePointPolytope(Point3D singlePoint)
   {
      ConvexPolytope polytope = new ConvexPolytope();
      polytope.addVertex(singlePoint, EPSILON);
      return polytope;
   }

   public static ConvexPolytope constructFromVertices(double[][] vertices)
   {
      ConvexPolytope polytope = new ConvexPolytope();
      for (double[] vertex : vertices)
      {
         polytope.addVertex(EPSILON, vertex);
      }
      return polytope;
   }
}
