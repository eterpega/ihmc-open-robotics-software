package us.ihmc.robotbuilder.gui;

import javafx.beans.property.Property;
import javafx.beans.property.SimpleObjectProperty;
import javafx.event.Event;
import javafx.geometry.Bounds;
import javafx.scene.*;
import javafx.scene.control.Button;
import javafx.scene.control.ToolBar;
import javafx.scene.control.Tooltip;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.Background;
import javafx.scene.layout.BackgroundFill;
import javafx.scene.layout.BorderPane;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Mesh;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.TriangleMesh;
import javafx.scene.shape.VertexFormat;
import javafx.scene.transform.Affine;
import javafx.scene.transform.MatrixType;
import javafx.scene.transform.Translate;
import org.jetbrains.annotations.Nullable;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.robotbuilder.util.*;
import us.ihmc.robotics.util.*;
import us.ihmc.robotics.util.TreeDifference.DifferencesByNode;
import us.ihmc.robotics.immutableRobotDescription.JointDescription;
import us.ihmc.robotics.immutableRobotDescription.graphics.*;

import javax.vecmath.Vector3d;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import java.util.stream.IntStream;
import java.util.stream.Stream;

import static java.util.Collections.emptyList;
import static java.util.stream.Collectors.toList;
import static us.ihmc.robotics.util.FunctionalObservableValue.functional;
import static us.ihmc.robotics.util.Memoization.memoized;

/**
 *
 */
@SuppressWarnings("OptionalUsedAsFieldOrParameterType")
public class Preview3D extends BorderPane
{
   private final Property<Optional<TreeFocus<Tree<JointDescription>>>> jointTree = new SimpleObjectProperty<>(Optional.empty());
   private final Property<Optional<PerspectiveCamera>> camera = new SimpleObjectProperty<>();

   private Optional<Graphics3DNode> sceneRoot = Optional.empty();
   private Optional<Graphics3DNode> highlightedNode = Optional.empty();
   private final Group subSceneRootGroup = new Group();
   private final SubScene subScene = new SubScene(subSceneRootGroup, 0, 0, true, SceneAntialiasing.BALANCED);
   private final ToolBar toolBar = new ToolBar();

   private static Preview3D instance;

   public static Preview3D getInstance() {
      return instance;
   }

   private final TreeMapping<Tree<JointDescription>, Graphics3DNode> treeMapping = new TreeMapping<>(this::treeMapper);

   public Preview3D()
   {
      instance = this;

      FunctionalObservableValue.combineLatest(widthProperty(), heightProperty())
                               .consume(dimension -> {
               subScene.widthProperty().setValue(dimension._1);
               subScene.heightProperty().setValue(dimension._2.doubleValue() - toolBar.getHeight());
            });
      subScene.setFocusTraversable(true);
      subScene.setManaged(false);

      setCenter(subScene);

      camera.addListener((observable, oldValue, newValue) -> newValue.ifPresent(newCamera -> {
         double locationX = newCamera.getLocalToSceneTransform().getTx();
         double locationY = newCamera.getLocalToSceneTransform().getTy();
         double locationZ = newCamera.getLocalToSceneTransform().getTz();
         Vector3d up = new Vector3d(0, 0, 1);
         Vector3d forward = new Vector3d(1, 0, 0);
         FocusBasedCameraMouseEventHandler cameraController = new FocusBasedCameraMouseEventHandler(subScene.widthProperty(), subScene.heightProperty(), newCamera, up, forward);

         Optional<Bounds> boundsOpt = sceneRoot.map(Node::getBoundsInParent);
         cameraController.getTranslate().setX(boundsOpt.map(bbox -> 0.5 * (bbox.getMinX() + bbox.getMaxX())).orElse(0.0));
         cameraController.getTranslate().setY(boundsOpt.map(bbox -> 0.5 * (bbox.getMinY() + bbox.getMaxY())).orElse(0.0));
         cameraController.getTranslate().setZ(boundsOpt.map(bbox -> 0.5 * (bbox.getMinZ() + bbox.getMaxZ())).orElse(0.0));
         cameraController.changeCameraPosition(locationX, locationY, locationZ);
         subScene.setCamera(newValue.orElse(null));
         subScene.addEventHandler(Event.ANY, cameraController);
      }));



      functional(jointTreeProperty())
            .flatMapOptional(Function.identity())
            .map(treeFocus -> treeFocus.root().getFocusedNode())
            .consume(newSceneRoot -> {
               boolean createDefaultCamera = !sceneRoot.isPresent();
               setRootJoint(newSceneRoot);
               camera.setValue(sceneRoot.filter(x -> createDefaultCamera).map(Preview3D::createDefaultCamera));
            });

      jointTreeProperty().addListener((observable, oldValue, newValue) ->
                                      {
                                         highlightedNode
                                               .ifPresent(graphicsNode -> highlightNode(graphicsNode, false));
                                         newValue.map(TreeFocus::getFocusedNode).flatMap(treeMapping::get)
                                                 .ifPresent(graphicsNode -> {
                                                    highlightNode(graphicsNode, true);
                                                    highlightedNode = Optional.of(graphicsNode);
                                                 });
                                      });

      setBackground(new Background(new BackgroundFill(new Color(0.1, 0.1, 0.1, 1), null, null)));
      createToolbar();
   }

   private Graphics3DNode treeMapper(Tree<JointDescription> node, List<Graphics3DNode> children)
   {
      Graphics3DNode result = new Graphics3DNode(node, children);
      applyDifferencesToItem(result, null, node.getValue());
      return result;
   }

   private static void highlightNode(Graphics3DNode node, boolean highLight)
   {
      Tree<Node> adaptedTree = Tree.adapt(node.graphicsGroup, n -> n instanceof Group ? ((Group)n).getChildren() : emptyList());
      adaptedTree.stream()
                 .filter(x -> x instanceof HighlightMeshView)
                 .map(x -> (HighlightMeshView)x)
                 .forEach(meshView -> meshView.setMaterial(highLight ? meshView.highlightMaterial : meshView.originalMaterial));

   }

   public Property<Optional<TreeFocus<Tree<JointDescription>>>> jointTreeProperty()
   {
      return jointTree;
   }

   void handleGraphicsClick(MouseEvent event)
   {
      if (event.getSource() instanceof Group)
      {
         if (((Group) event.getSource()).getParent() instanceof Graphics3DNode)
         {
            Graphics3DNode clickedNode = (Graphics3DNode) ((Group) event.getSource()).getParent();
            sceneRoot.flatMap(root -> root.originalTree.find(node -> node == clickedNode.originalTree)).ifPresent(newRoot -> jointTree.setValue(Optional.of(newRoot)));
         }
      }
   }

   private void applyDifferencesByNode(Graphics3DNode parentItem, DifferencesByNode<JointDescription> differencesByNode)
   {
      differencesByNode.get(parentItem.getOriginalTree()).ifPresent(diff -> {
         parentItem.getChildrenSpecific()
                   .forEach(child -> applyDifferencesByNode(child, differencesByNode));

         treeMapping.replaceSingleNode(parentItem.getOriginalTree(), diff.getNewNode());
         JointDescription oldValue = parentItem.getOriginalTree().getValue();
         JointDescription newValue = diff.getNewNode().getValue();
         applyDifferencesToItem(parentItem, oldValue, newValue);

         parentItem.setOriginalTree(diff.getNewNode());
         parentItem.getChildren().addAll(diff.getAddedChildren().map(treeMapping::mapNewNode).toJavaList());
         diff.getRemovedChildren().forEach(treeMapping::removeMapping);
         parentItem.getChildren().removeIf(child -> {
            if (!(child instanceof Graphics3DNode))
               return false;
            Graphics3DNode graphicsChild = (Graphics3DNode) child;
            return diff.getRemovedChildren().contains(graphicsChild.getOriginalTree());
         });

         // Do not reorder children as we do not care much about the order
      });
   }

   private void applyDifferencesToItem(Graphics3DNode parentItem, @Nullable JointDescription oldValue, JointDescription newValue)
   {
      if (oldValue == null || !oldValue.getLink().getLinkGraphics().equals(newValue.getLink().getLinkGraphics()))
         parentItem.setGraphicsGroup(memoized(Preview3D::convertGraphicsGroup).apply(newValue.getLink().getLinkGraphics()));

      Vector3d oldOffset = oldValue == null ? new Vector3d() : oldValue.getOffsetFromJoint();
      if (!oldOffset.equals(newValue.getOffsetFromJoint()))
      {
         Vector3d diffTranslation = new Vector3d(newValue.getOffsetFromJoint());
         diffTranslation.sub(oldOffset);
         parentItem.getTransforms().add(new Translate(diffTranslation.x, diffTranslation.y, diffTranslation.z));
      }
   }

   /**
    * Change the root joint of this 3D tree view.
    * The new root will be diffed against the existing tree and
    * only necessary changes will be applied to the tree.
    * @param newRoot new root
    */
   private void setRootJoint(Tree<JointDescription> newRoot)
   {
      if (newRoot == null)
      {
         subSceneRootGroup.getChildren().clear();
         sceneRoot = Optional.empty();
         return;
      }

      sceneRoot.map((sceneRoot) ->
      {
         applyDifferencesByNode(sceneRoot, TreeDifference.difference(sceneRoot.getOriginalTree(), newRoot));
         return sceneRoot;
      }).orElseGet(() -> {
         Graphics3DNode newGraphicsRoot = treeMapping.mapNewNode(newRoot);
         this.sceneRoot = Optional.of(newGraphicsRoot);
         subSceneRootGroup.getChildren().setAll(Collections.singletonList(newGraphicsRoot));
         return null;
      });

   }

   private void createToolbar()
   {
      String[] titles = {"Bottom", "Front", "Left", "Rear", "Right", "Top"};
      String[] icons = {"boxBottom.png", "boxFront.png", "boxLeft.png", "boxRear.png", "boxRight.png", "boxTop.png"};
      Vector3d[] directions = {new Vector3d(0, 0, 1), new Vector3d(-1, 0, 0), new Vector3d(0, 1, 0), new Vector3d(1, 0, 0), new Vector3d(0, -1, 0), new Vector3d(0, 0, -1)};
      for (int i = 0; i < icons.length; i++)
      {
         Button button = new Button("", new ImageView(new Image(getClass().getResourceAsStream("/icons/" + icons[i]))));
         button.setTooltip(new Tooltip(titles[i]));
         final int buttonIndex = i;
         button.setOnAction(event -> sceneRoot.ifPresent(rootNode ->
                     camera.setValue(Optional.of(Util.lookAtNodeFromDirection(rootNode, 60, directions[buttonIndex], new Vector3d(0, 0, 1))))));
         toolBar.getItems().add(button);
      }
      setBottom(toolBar);
   }

   private static PerspectiveCamera createDefaultCamera(Node nodeToLookAt)
   {
      return Util.lookAtNodeFromDirection(nodeToLookAt, 60, new Vector3d(-1, 0, 0), new Vector3d(0, 0, 1));
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
      PhongMaterial material = memoized(Preview3D::convertMaterial).apply(geometryDescription.getMaterial());
      PhongMaterial highlightMaterial = new PhongMaterial(material.getDiffuseColor().interpolate(Color.LIGHTBLUE, 0.85));
      HighlightMeshView meshView = new HighlightMeshView(memoized(Preview3D::convertMesh).apply(sourceMesh), material, highlightMaterial);
      geometryGroup.getChildren().add(meshView);
      geometryGroup.getTransforms().add(memoized(Preview3D::convertTransform).apply(geometryDescription.getTransform()));
      return geometryGroup;
   }

   private static Group convertGraphicsGroup(GraphicsGroupDescription groupDescription)
   {
      Group graphicsGroup = new Group();
      graphicsGroup.getTransforms().add(memoized(Preview3D::convertTransform).apply(groupDescription.getTransform()));
      graphicsGroup.getChildren().addAll(groupDescription.getChildGeometries().stream().map(memoized(Preview3D::convertGeometry)).collect(toList()));
      graphicsGroup.getChildren().addAll(groupDescription.getChildGroups().stream().map(memoized(Preview3D::convertGraphicsGroup)).collect(toList()));
      return graphicsGroup;
   }

   private static Affine convertTransform(TransformDescription transformDescription)
   {
      return new Affine(transformDescription.toDoubleArrayRowMajor(), MatrixType.MT_3D_4x4, 0);
   }

   private class Graphics3DNode extends Group
   {
      private Tree<JointDescription> originalTree;
      private @Nullable Group graphicsGroup;
      private final Tooltip tooltip;

      Graphics3DNode(Tree<JointDescription> originalTree, List<Graphics3DNode> children)
      {
         this.originalTree = originalTree;
         getChildren().addAll(children);
         tooltip = new Tooltip(originalTree.getValue().getName());
      }

      Tree<JointDescription> getOriginalTree()
      {
         return originalTree;
      }

      void setOriginalTree(Tree<JointDescription> originalTree)
      {
         this.originalTree = originalTree;
         tooltip.setText(originalTree.getValue().getName());
      }

      Stream<Graphics3DNode> getChildrenSpecific()
      {
         return getChildren().stream()
                             .flatMap(child -> child instanceof Graphics3DNode ? Stream.of((Graphics3DNode)child) : Stream.empty());
      }

      void setGraphicsGroup(@Nullable Group graphicsGroup)
      {
         getChildren().remove(this.graphicsGroup);
         getChildren().add(graphicsGroup);
         this.graphicsGroup = graphicsGroup;
         if (graphicsGroup != null && !graphicsGroup.getBoundsInLocal().isEmpty())
         {
            Tooltip.install(graphicsGroup, tooltip);
            graphicsGroup.addEventHandler(MouseEvent.MOUSE_CLICKED, Preview3D.this::handleGraphicsClick);
         }
      }

      @Override public String toString()
      {
         return "Graphics of " + originalTree.toString();
      }
   }

   private static class HighlightMeshView extends MeshView
   {
      private final PhongMaterial originalMaterial, highlightMaterial;

      HighlightMeshView(Mesh mesh, PhongMaterial originalMaterial, PhongMaterial highlightMaterial)
      {
         super(mesh);
         this.originalMaterial = originalMaterial;
         this.highlightMaterial = highlightMaterial;
         setMaterial(originalMaterial);
      }
   }
}
