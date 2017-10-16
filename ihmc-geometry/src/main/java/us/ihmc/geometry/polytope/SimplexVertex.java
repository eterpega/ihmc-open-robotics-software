package us.ihmc.geometry.polytope;

public class SimplexVertex extends ExtendedPolytopeVertex
{
   ExtendedPolytopeVertex polytopeAVertexReference;
   ExtendedPolytopeVertex polytopeBVertexReference;
   
   public SimplexVertex()
   {
      super();
   }
   
   public SimplexVertex(ExtendedPolytopeVertex vertexOnPolytopeA, ExtendedPolytopeVertex vertexOnPolytopeB)
   {
      set(vertexOnPolytopeA, vertexOnPolytopeB);
   }
   
   public void set(ExtendedPolytopeVertex vertexOnPolytopeA, ExtendedPolytopeVertex vertexOnPolytopeB)
   {
      this.polytopeAVertexReference = vertexOnPolytopeA;
      this.polytopeBVertexReference = vertexOnPolytopeB;
      sub(vertexOnPolytopeA, vertexOnPolytopeB);
   }
   
   public ExtendedPolytopeVertex getVertexOnPolytopeA()
   {
      return polytopeAVertexReference;
   }
   
   public ExtendedPolytopeVertex getVertexOnPolytopeB()
   {
      return polytopeBVertexReference;
   }
   
   
}
