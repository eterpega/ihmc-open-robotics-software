package us.ihmc.geometry.polytope;

import static org.junit.Assert.*;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.commons.Epsilons;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FrameConvexPolytope;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FramePolytopeVertex;

public class FrameConvexPolytopeTest
{
   private static final double epsilon = Epsilons.ONE_BILLIONTH;
   private RigidBodyTransform transformFromParent;
   private ReferenceFrame boxFrame;

   @Before
   public void setupTest()
   {
      transformFromParent = new RigidBodyTransform();
      transformFromParent.setTranslation(1, 1, 1);
      boxFrame = ReferenceFrame.constructFrameWithUnchangingTransformFromParent("BoxFrame", ReferenceFrame.getWorldFrame(), transformFromParent);
   }
   
   @After
   public void tearDownTest()
   {
      
   }
   
   @Test
   public void testConstructor()
   {
      FrameConvexPolytope framePolytope = new FrameConvexPolytope(boxFrame);
      framePolytope.addVertex(0, 0, 0, epsilon);
      framePolytope.addVertex(1, 0, 0, epsilon);
      framePolytope.addVertex(1, 1, 0, epsilon);
      framePolytope.addVertex(0, 1, 0, epsilon);

      framePolytope.addVertex(0, 0, 1, epsilon);
      framePolytope.addVertex(1, 0, 1, epsilon);
      framePolytope.addVertex(1, 1, 1, epsilon);
      framePolytope.addVertex(0, 1, 1, epsilon);
      
      PrintTools.debug("");
   }
   
   @Test
   public void testVertexConstruction()
   {
      FramePolytopeVertex vertex1 = new FramePolytopeVertex(boxFrame);
      assertTrue(vertex1 != null);
      assertTrue(vertex1.getReferenceFrame() == boxFrame);
      assertTrue(vertex1.getPosition().getX() == 0.0);
      assertTrue(vertex1.getPosition().getY() == 0.0);
      assertTrue(vertex1.getPosition().getZ() == 0.0);
   }
}
