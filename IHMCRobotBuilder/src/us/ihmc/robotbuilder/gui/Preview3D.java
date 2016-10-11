package us.ihmc.robotbuilder.gui;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.scene.*;
import javafx.scene.layout.Background;
import javafx.scene.layout.BackgroundFill;
import javafx.scene.layout.GridPane;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.TriangleMesh;
import javafx.scene.shape.VertexFormat;
import javafx.scene.transform.Affine;
import javafx.scene.transform.MatrixType;
import javafx.scene.transform.Translate;
import us.ihmc.robotbuilder.util.*;
import us.ihmc.robotbuilder.util.TreeInterface.CachedMapper;
import us.ihmc.robotbuilder.util.TreeInterface.TreeNodeMapper;
import us.ihmc.robotics.immutableRobotDescription.JointDescription;
import us.ihmc.robotics.immutableRobotDescription.graphics.*;

import javax.vecmath.Vector3d;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import java.util.stream.Stream;

/**
 *
 */
@SuppressWarnings("OptionalUsedAsFieldOrParameterType")
public class Preview3D extends GridPane
{
   private final Property<Optional<TreeFocus<Tree<JointDescription>>>> jointTree = new SimpleObjectProperty<>(Optional.empty());
   private final Property<Optional<PerspectiveCamera>> camera = new SimpleObjectProperty<>();

   private Optional<Group> sceneRoot = Optional.empty();
   private final SubScene subScene = new SubScene(new Group(), 0, 0, true, SceneAntialiasing.BALANCED);

   private CachedMapper<Tree<JointDescription>, Group> treeToNodeCachedMapper;

   public Preview3D()
   {
      subScene.widthProperty().bind(widthProperty());
      subScene.heightProperty().bind(heightProperty());

      add(subScene, 1, 1);

      camera.addListener((observable, oldValue, newValue) -> subScene.setCamera(newValue.orElse(null)));

      FunctionalObservableValue.of(jointTreeProperty())
            .flatMapOptional(Function.identity())
            .map(treeFocus -> treeFocus.root().getFocusedNode())
            .map(this::convertTree)
            .consume(newSceneRoot -> {
               if (!sceneRoot.isPresent())
                  camera.setValue(Optional.of(createDefaultCamera(newSceneRoot)));
               sceneRoot = Optional.of(newSceneRoot);
               subScene.setRoot(newSceneRoot);
            });

      setBackground(new Background(new BackgroundFill(new Color(0.1, 0.1, 0.1, 1), null, null)));
   }

   public Property<Optional<TreeFocus<Tree<JointDescription>>>> jointTreeProperty()
   {
      return jointTree;
   }

   public void setTree(Tree<JointDescription> tree)
   {
      jointTree.setValue(Optional.of(tree.getFocus()));
   }

   private static PerspectiveCamera createDefaultCamera(Node nodeToLookAt)
   {
      return Util.lookAtNodeFromDirection(nodeToLookAt, 60, new Vector3d(1, 0, 0), new Vector3d(0, 1, 0));
   }

   private static PhongMaterial convertMaterial(MaterialDescription materialDescription)
   {
      PhongMaterial result = new PhongMaterial();
      result.setDiffuseColor(new Color(
            materialDescription.getColor().getRed() / 255f,
            materialDescription.getColor().getGreen() / 255f,
            materialDescription.getColor().getBlue() / 255f,
            materialDescription.getColor().getAlpha() / 255f
      ));
      return result;
   }

   private static Mesh convertMesh(TriangleMeshDescription sourceMesh)
   {
      TriangleMesh targetMesh = sourceMesh.getNormalBuffer() == null ? new TriangleMesh(VertexFormat.POINT_TEXCOORD) : new TriangleMesh(VertexFormat.POINT_NORMAL_TEXCOORD);
      targetMesh.getPoints().addAll(Arrays.stream(sourceMesh.getVertexBuffer())
                                    .flatMap(vertex -> Stream.of(vertex.x, vertex.y, vertex.z))
                                    .collect(FloatArrayCollector.create()));
      final int indices = 1 + (sourceMesh.getTextureCoordinatesBuffer() != null ? 1 : 0) + (sourceMesh.getNormalBuffer() != null ? 1 : 0);
      targetMesh.getFaces().addAll(Arrays.stream(sourceMesh.getIndexBuffer())
                                   .flatMap(x -> IntStream.generate(() -> x).limit(indices)) // one index per each vertex/normal/texture buffer
                                   .toArray());
      if (sourceMesh.getTextureCoordinatesBuffer() != null)
         targetMesh.getTexCoords().addAll(Arrays.stream(sourceMesh.getTextureCoordinatesBuffer())
                                          .flatMap(texCoord -> Stream.of(texCoord.x, texCoord.y))
                                          .collect(FloatArrayCollector.create())
         );

      if (sourceMesh.getNormalBuffer() != null)
         targetMesh.getNormals().addAll(Arrays.stream(sourceMesh.getNormalBuffer())
                                        .flatMap(normal -> Stream.of(normal.x, normal.y, normal.z))
                                        .collect(FloatArrayCollector.create()));

      return targetMesh;
   }

   private static Node convertGeometry(GeometryDescription geometryDescription)
   {
      Group geometryGroup = new Group();
      TriangleMeshDescription sourceMesh = geometryDescription.toTriangleGeometry().getTriangleMesh();
      MeshView meshView = new MeshView(convertMesh(sourceMesh));
      meshView.setMaterial(convertMaterial(geometryDescription.getMaterial()));
      geometryGroup.getChildren().add(meshView);
      geometryGroup.getTransforms().add(convertTransform(geometryDescription.getTransform()));
      return geometryGroup;
   }

   private static Node convertGraphicsGroup(GraphicsGroupDescription groupDescription)
   {
      Group graphicsGroup = new Group();
      graphicsGroup.getTransforms().add(convertTransform(groupDescription.getTransform()));
      graphicsGroup.getChildren().addAll(groupDescription.getChildGeometries().stream().map(Preview3D::convertGeometry).collect(Collectors.toList()));
      graphicsGroup.getChildren().addAll(groupDescription.getChildGroups().stream().map(Preview3D::convertGraphicsGroup).collect(Collectors.toList()));
      return graphicsGroup;
   }

   private static Affine convertTransform(TransformDescription transformDescription)
   {
      return new Affine(transformDescription.toDoubleArrayRowMajor(), MatrixType.MT_3D_4x4, 0);
   }

   private Group convertTree(Tree<JointDescription> tree)
   {
      if (treeToNodeCachedMapper == null)
      {
         treeToNodeCachedMapper = TreeInterface.cachedMap(new TreeNodeMapper<Tree<JointDescription>, Group>()
         {
            @Override public Group mapNode(Tree<JointDescription> node, List<Group> children)
            {
               Group result = new Group();
               Vector3d offset = node.getValue().getOffsetFromJoint();
               result.getTransforms().add(new Translate(offset.x, offset.y, offset.z));
               result.getChildren().add(convertGraphicsGroup(node.getValue().getLink().getLinkGraphics()));
               result.getChildren().addAll(children);
               return result;
            }
         });
      }

      //return treeToNodeCachedMapper.map(tree);
      return tree.map(new TreeNodeMapper<Tree<JointDescription>, Group>()
      {
         @Override public Group mapNode(Tree<JointDescription> node, List<Group> children)
         {
            Group result = new Group();
            Vector3d offset = node.getValue().getOffsetFromJoint();
            result.getTransforms().add(new Translate(offset.x, offset.y, offset.z));
            result.getChildren().add(convertGraphicsGroup(node.getValue().getLink().getLinkGraphics()));
            result.getChildren().addAll(children);
            return result;
         }
      });
   }

}
