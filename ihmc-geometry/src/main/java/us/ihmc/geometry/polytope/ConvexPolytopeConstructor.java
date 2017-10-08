package us.ihmc.geometry.polytope;

import java.util.Random;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class ConvexPolytopeConstructor
{
   private static final double EPSILON = Epsilons.ONE_MILLIONTH;
   
   public static ConvexPolytope constructUnitCube()
   {
      ConvexPolytope polytope = new ConvexPolytope();
      
      
      return polytope;
   }

   public static ConvexPolytope constructBoxWithCenterAtZero(double halfLengthX, double halfWidthY, double halfHeightZ)
   {
      ConvexPolytope polytope = new ConvexPolytope();

      polytope.addVertex(new PolytopeVertex(-halfLengthX, -halfWidthY, -halfHeightZ), EPSILON);
      PrintTools.debug(polytope.toString());
      polytope.addVertex(new PolytopeVertex(halfLengthX, -halfWidthY, -halfHeightZ), EPSILON);
      PrintTools.debug(polytope.toString());
      polytope.addVertex(new PolytopeVertex(halfLengthX, halfWidthY, -halfHeightZ), EPSILON);
      PrintTools.debug(polytope.toString());
      polytope.addVertex(new PolytopeVertex(-halfLengthX, halfWidthY, -halfHeightZ), EPSILON);
      PrintTools.debug(polytope.toString());
      polytope.addVertex(new PolytopeVertex(-halfLengthX, -halfWidthY, halfHeightZ), EPSILON);
      PrintTools.debug(polytope.toString());
      polytope.addVertex(new PolytopeVertex(halfLengthX, -halfWidthY, halfHeightZ), EPSILON);
      PrintTools.debug(polytope.toString());
      polytope.addVertex(new PolytopeVertex(halfLengthX, halfWidthY, halfHeightZ), EPSILON);
      PrintTools.debug(polytope.toString());
      polytope.addVertex(new PolytopeVertex(-halfLengthX, halfWidthY, halfHeightZ), EPSILON);
      PrintTools.debug(polytope.toString());

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

         //TODO: Need to connect the edges later once they are used in the algorithm!!
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
