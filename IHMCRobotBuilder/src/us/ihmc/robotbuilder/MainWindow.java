package us.ihmc.robotbuilder;

import javafx.application.Application;
import javafx.collections.ListChangeListener;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.geometry.Rectangle2D;
import javafx.scene.*;
import javafx.scene.control.*;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.input.KeyEvent;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.Sphere;
import javafx.scene.shape.TriangleMesh;
import javafx.scene.shape.VertexFormat;
import javafx.scene.transform.Affine;
import javafx.scene.transform.MatrixType;
import javafx.scene.transform.Translate;
import javafx.stage.FileChooser;
import javafx.stage.Screen;
import javafx.stage.Stage;
import javaslang.control.Option;
import us.ihmc.javaFXToolkit.cameraControllers.SimpleCameraKeyboardEventHandler;
import us.ihmc.javaFXToolkit.cameraControllers.SimpleCameraMouseEventHandler;
import us.ihmc.robotbuilder.gui.JointEditorPane;
import us.ihmc.robotbuilder.model.JointWrapper;
import us.ihmc.robotbuilder.model.Loader;
import us.ihmc.robotbuilder.util.*;
import us.ihmc.robotics.immutableRobotDescription.JointDescription;
import us.ihmc.robotics.immutableRobotDescription.RobotDescription;
import us.ihmc.robotics.immutableRobotDescription.graphics.GeometryDescription;
import us.ihmc.robotics.immutableRobotDescription.graphics.GraphicsGroupDescription;
import us.ihmc.robotics.immutableRobotDescription.graphics.MaterialDescription;
import us.ihmc.robotics.immutableRobotDescription.graphics.TriangleMeshDescription;

import javax.vecmath.Vector3d;
import java.io.File;
import java.util.Arrays;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import java.util.stream.Stream;

/**
 *
 */
public class MainWindow extends Application {
    @FXML
    private ScrollPane jointSettings;

    @FXML
    private Pane view3D;

    @FXML
    private TreeView<JointWrapper> treeView;

    private Stage stage;

    @Override
    public void start(Stage primaryStage) throws Exception {
        Parent root = FXMLLoader.load(getClass().getResource("/main_window.fxml"));

        Screen screen = Screen.getPrimary();
        Rectangle2D bounds = screen.getVisualBounds();

        primaryStage.setX(bounds.getMinX());
        primaryStage.setY(bounds.getMinY());
        primaryStage.setWidth(bounds.getWidth());
        primaryStage.setHeight(bounds.getHeight());
        Scene scene = new Scene(root, bounds.getWidth(), bounds.getHeight());

        this.stage = primaryStage;
        stage.setTitle("IHMC Robot Builder");
        stage.setScene(scene);
        stage.setMaximized(true);
        stage.show();
    }

    /**
     * File -> Open action
     */
    public void onOpen() {
        FileChooser chooser = new FileChooser();
        chooser.setTitle("Open Robot Definition File");
        if (System.getProperty("initial.dir") != null)
            chooser.setInitialDirectory(new File(System.getProperty("initial.dir")));
        File file = chooser.showOpenDialog(stage);
        if (file == null)
            return;

        Loader.loadFile(file, options -> Util.runLaterInUI(() -> {
            if (options.isEmpty())
                return Option.none();
            if (options.length() == 1)
                return Option.of(options.get(0));

            // Let the user choose a model to load
            ChoiceDialog<String> dialog = new ChoiceDialog<>(options.get(0), options.toJavaArray(String.class));
            dialog.setTitle("Model Selection");
            dialog.setHeaderText("Please choose a model to open.");
            dialog.setContentText("Model:");
            return Option.ofOptional(dialog.showAndWait());
        })).flatMap(immutableRobotDescription -> Util.runLaterInUI(() -> {
            immutableRobotDescription.peek(description -> {
                Tree<JointWrapper> tree = Tree.adapt(description, JointDescription::getChildrenJoints)
                                              .mapValues(JointWrapper::new);
                treeView.setRoot(tree.map((node, children) -> {
                    TreeItem<JointWrapper> item = new TreeItem<>(node.getValue());
                    item.getChildren().addAll(children);
                    return item;
                }));

                treeView.getSelectionModel()
                        .getSelectedItems()
                        .addListener((ListChangeListener<? super TreeItem<JointWrapper>>) change ->
                        {
                            while (change.next())
                            {
                                if (!change.wasAdded())
                                    continue;
                                change.getAddedSubList().forEach(selected -> jointSettings.setContent(new JointEditorPane(selected.getValue().getJointDescription())));
                            }
                        });

                populate3DView(description);
            });
            return null;
        })).onFailure(err -> Util.runLaterInUI(() -> {
            Alert alert = new Alert(AlertType.ERROR, "Error loading file: " + err.getMessage(), ButtonType.OK);
            alert.showAndWait();
            err.printStackTrace();
            return null;
        }));
    }

    private void populate3DView(RobotDescription description) {
        Group group = new Group();
        description.getChildrenJoints().stream().map(this::create3DNodes).forEach(group.getChildren()::add);

        SubScene scene3d = new SubScene(group, view3D.getWidth(), view3D.getHeight(), true, SceneAntialiasing.DISABLED);
        scene3d.setFill(Color.BLACK);
        PerspectiveCamera camera = Util.lookAtNodeFromDirection(group, 60, new Vector3d(1, 0, 0), new Vector3d(0, 1, 0));
        scene3d.setCamera(camera);

        SimpleCameraMouseEventHandler mouseController = new SimpleCameraMouseEventHandler(camera);
        SimpleCameraKeyboardEventHandler keyboardController = new SimpleCameraKeyboardEventHandler(camera);
        scene3d.addEventHandler(MouseEvent.ANY, mouseController);
        scene3d.addEventHandler(KeyEvent.ANY, keyboardController);
        scene3d.setFocusTraversable(true);
        scene3d.requestFocus();

        view3D.getChildren().add(scene3d);
    }

    private Node create3DNodes(JointDescription description) {
        Tree<JointDescription> tree = Tree.adapt(description, JointDescription::getChildrenJoints);
        return tree.map((node, children) -> {
                Group jointGroup = new Group();
                Vector3d offset = node.getValue().getOffsetFromJoint();
                jointGroup.getTransforms().add(new Translate(offset.x, offset.y, offset.z));

                final PhongMaterial redMaterial = new PhongMaterial();
                redMaterial.setDiffuseColor(Color.DARKRED);
                redMaterial.setSpecularColor(Color.RED);

                Sphere graphics = new Sphere(0.05);
                graphics.setMaterial(redMaterial);
                jointGroup.getChildren().add(graphics);

                jointGroup.getChildren().add(convertGraphicsDescription(node.getValue().getLink().getLinkGraphics()));

                jointGroup.getChildren().addAll(children);
                return jointGroup;
        });
    }

    private Node convertGraphicsDescription(GraphicsGroupDescription groupDescription) {
        Group graphicsGroup = new Group();
        graphicsGroup.getTransforms().add(new Affine(groupDescription.getTransform().toDoubleArrayRowMajor(), MatrixType.MT_3D_4x4, 0));
        graphicsGroup.getChildren().addAll(groupDescription.getChildGeometries().stream().map(this::convertGeometryDescription).collect(Collectors.toList()));
        graphicsGroup.getChildren().addAll(groupDescription.getChildGroups().stream().map(this::convertGraphicsDescription).collect(Collectors.toList()));
        return graphicsGroup;
    }

    private Node convertGeometryDescription(GeometryDescription geometryDescription) {
        Group geometryGroup = new Group();
        TriangleMeshDescription sourceMesh = geometryDescription.toTriangleGeometry().getTriangleMesh();
        TriangleMesh mesh = sourceMesh.getNormalBuffer() == null ? new TriangleMesh(VertexFormat.POINT_TEXCOORD) : new TriangleMesh(VertexFormat.POINT_NORMAL_TEXCOORD);
        mesh.getPoints().addAll(Arrays.stream(sourceMesh.getVertexBuffer())
                                      .flatMap(vertex -> Stream.of(vertex.x, vertex.y, vertex.z))
                                      .collect(FloatArrayCollector.create()));
        final int indices = 1 + (sourceMesh.getTextureCoordinatesBuffer() != null ? 1 : 0) + (sourceMesh.getNormalBuffer() != null ? 1 : 0);
        mesh.getFaces().addAll(Arrays.stream(sourceMesh.getIndexBuffer())
                                     .flatMap(x -> IntStream.generate(() -> x).limit(indices)) // one index per each vertex/normal/texture buffer
                                     .toArray());
        if (sourceMesh.getTextureCoordinatesBuffer() != null)
            mesh.getTexCoords().addAll(Arrays.stream(sourceMesh.getTextureCoordinatesBuffer())
                                            .flatMap(texCoord -> Stream.of(texCoord.x, texCoord.y))
                                            .collect(FloatArrayCollector.create())
            );

        if (sourceMesh.getNormalBuffer() != null)
            mesh.getNormals().addAll(Arrays.stream(sourceMesh.getNormalBuffer())
                                           .flatMap(normal -> Stream.of(normal.x, normal.y, normal.z))
                                           .collect(FloatArrayCollector.create()));

        MeshView meshView = new MeshView(mesh);
        meshView.setMaterial(convertMaterial(geometryDescription.getMaterial()));
        geometryGroup.getChildren().add(meshView);
        geometryGroup.getTransforms().add(new Affine(geometryDescription.getTransform().toDoubleArrayRowMajor(), MatrixType.MT_3D_4x4, 0));
        return geometryGroup;
    }

    private Material convertMaterial(MaterialDescription materialDescription) {
        PhongMaterial result = new PhongMaterial();
        result.setDiffuseColor(new Color(
              materialDescription.getColor().getRed() / 255f,
              materialDescription.getColor().getGreen() / 255f,
              materialDescription.getColor().getBlue() / 255f,
              materialDescription.getColor().getAlpha() / 255f
        ));
        return result;
    }
}
