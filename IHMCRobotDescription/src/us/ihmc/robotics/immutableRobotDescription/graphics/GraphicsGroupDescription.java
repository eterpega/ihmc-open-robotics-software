package us.ihmc.robotics.immutableRobotDescription.graphics;

import org.immutables.value.Value.Default;
import org.immutables.value.Value.Immutable;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.instructions.*;
import us.ihmc.graphicsDescription.instructions.primitives.Graphics3DIdentityInstruction;
import us.ihmc.graphicsDescription.instructions.primitives.Graphics3DRotateInstruction;
import us.ihmc.graphicsDescription.instructions.primitives.Graphics3DScaleInstruction;
import us.ihmc.graphicsDescription.instructions.primitives.Graphics3DTranslateInstruction;

import java.util.Collections;
import java.util.List;

/**
 * Allows grouping multiple {@link GeometryDescription} object into one.
 */
@Immutable
public abstract class GraphicsGroupDescription implements Transformable
{
   private static final GraphicsGroupDescription[] EMPTY_GROUP = new GraphicsGroupDescription[1];

   public static GraphicsGroupDescription empty()
   {
      if (EMPTY_GROUP[0] == null)
         EMPTY_GROUP[0] = GraphicsGroupDescription.builder()
                                               .childGeometries(Collections.<GeometryDescription>emptyList())
                                               .childGroups(Collections.<GraphicsGroupDescription>emptyList())
                                               .build();
      return EMPTY_GROUP[0];
   }

   public abstract List<GraphicsGroupDescription> getChildGroups();

   public abstract List<GeometryDescription> getChildGeometries();

   @Default
   public TransformDescription getTransform()
   {
      return TransformDescription.IDENTITY;
   }

   public static GraphicsGroupDescription fromGraphics3DObject(Graphics3DObject graphics3DObject) {
      ImmutableGraphicsGroupDescription.Builder groupBuilder = GraphicsGroupDescription.builder();
      TransformDescription currentTransform = new TransformDescription();
      for (Graphics3DPrimitiveInstruction instruction : graphics3DObject.getGraphics3DInstructions())
      {
         if (instruction instanceof Graphics3DAddModelFileInstruction)
         {
            // TODO: implement model file loading that's independent of JMonkey/JavaFX
            System.err.println("WARNING: model file loading not supported in GraphicsGroupDescription");
         }
         else if (instruction instanceof Graphics3DAddMeshDataInstruction)
         {
            Graphics3DAddMeshDataInstruction graphics3DAddMesh = (Graphics3DAddMeshDataInstruction) instruction;
            TriangleMeshDescription mesh = TriangleMeshDescription
                  .fromTriangleSoupSanitized(TriangleSoupDescription.fromMeshDataHolder(graphics3DAddMesh.getMeshData()));
            TriangleGeometryDescription geometry = TriangleGeometryDescription.builder()
                                                                           .triangleMesh(mesh)
                                                                           .transform(currentTransform)
                                                                           .build();
            groupBuilder.addChildGeometries(geometry);
         }
         else if (instruction instanceof Graphics3DIdentityInstruction)
         {
            currentTransform = new TransformDescription();
         }
         else if (instruction instanceof Graphics3DRotateInstruction)
         {
            Graphics3DRotateInstruction graphics3DRotateMatrix = (Graphics3DRotateInstruction) instruction;
            currentTransform = currentTransform.compose(TransformDescription.fromRotation(graphics3DRotateMatrix.getRotationMatrix()));
         }
         else if (instruction instanceof Graphics3DScaleInstruction)
         {
            Graphics3DScaleInstruction graphics3DScale = (Graphics3DScaleInstruction) instruction;
            currentTransform = currentTransform.compose(TransformDescription.fromScale(graphics3DScale.getScaleFactor()));
         }
         else if (instruction instanceof Graphics3DTranslateInstruction)
         {
            Graphics3DTranslateInstruction graphics3DTranslate = (Graphics3DTranslateInstruction) instruction;
            currentTransform = currentTransform.compose(TransformDescription.fromTranslation(graphics3DTranslate.getTranslation()));
         }
         else if (instruction instanceof Graphics3DAddExtrusionInstruction)
         {
            // TODO: implement extrusion
            System.err.println("WARNING: extrusion not supported in GraphicsGroupDescription");
         }
         else if (instruction instanceof Graphics3DAddHeightMapInstruction)
         {
            Graphics3DAddHeightMapInstruction graphics3DAddHeightMap = (Graphics3DAddHeightMapInstruction) instruction;
            HeightMapDescription heightMap = HeightMapDescription.builder().heightMap(graphics3DAddHeightMap.getHeightMap()).transform(currentTransform)
                                                                              .build();
            groupBuilder.addChildGeometries(heightMap);
         }
         else
         {
            System.err.println("Unknown graphics3DDefinition: " + instruction.getClass().getSimpleName());
         }

      }
      return groupBuilder.build();
   }

   public static ImmutableGraphicsGroupDescription.Builder builder()
   {
      return ImmutableGraphicsGroupDescription.builder();
   }
}
