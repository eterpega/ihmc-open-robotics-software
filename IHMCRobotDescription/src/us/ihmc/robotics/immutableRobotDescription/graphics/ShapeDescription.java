package us.ihmc.robotics.immutableRobotDescription.graphics;

/**
 * Represents a shape in a 3D environment
 */
public interface ShapeDescription
{
   /**
    * Converts this 3D shape to a triangle mesh usable for rendering
    * @return triangle mesh representation of this shape
    */
   TriangleMeshDescription toTriangleMesh();
}
