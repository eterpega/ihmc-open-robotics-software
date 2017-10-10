package us.ihmc.geometry.polytope;

public class SimplexVertex extends PolytopeVertex
{
   PolytopeVertex polytopeAVertexReference;
   PolytopeVertex polytopeBVertexReference;
   
   public SimplexVertex()
   {
      super();
   }
   
   public SimplexVertex(PolytopeVertex vertexOnPolytopeA, PolytopeVertex vertexOnPolytopeB)
   {
      this.polytopeAVertexReference = vertexOnPolytopeA;
      this.polytopeBVertexReference = vertexOnPolytopeB;
      sub(vertexOnPolytopeA, vertexOnPolytopeB);
   }
   
   public void set(PolytopeVertex vertexOnPolytopeA, PolytopeVertex vertexOnPolytopeB)
   {
      this.polytopeAVertexReference = vertexOnPolytopeA;
      this.polytopeBVertexReference = vertexOnPolytopeB;
      sub(vertexOnPolytopeA, vertexOnPolytopeB);
   }
   
   public PolytopeVertex getVertexOnPolytopeA()
   {
      return polytopeAVertexReference;
   }
   
   public PolytopeVertex getVertexOnPolytopeB()
   {
      return polytopeBVertexReference;
   }
   
   
}
