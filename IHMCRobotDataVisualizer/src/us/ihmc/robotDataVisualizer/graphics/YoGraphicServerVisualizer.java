package us.ihmc.robotDataVisualizer.graphics;

import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelLoader;
import us.ihmc.multicastLogDataProtocol.modelLoaders.SDFModelLoader;
import us.ihmc.robotDataLogger.YoVariableClient;
import us.ihmc.robotDataLogger.YoVariablesUpdatedListener;
import us.ihmc.robotDataLogger.handshake.LogHandshake;
import us.ihmc.robotDataLogger.handshake.YoVariableHandshakeParser;
import us.ihmc.robotDataLogger.jointState.JointState;
import us.ihmc.robotDataVisualizer.visualizer.JointUpdater;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.yoVariables.dataBuffer.DataBuffer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.ArrayList;
import java.util.List;

public class YoGraphicServerVisualizer extends Application implements YoVariablesUpdatedListener
{
   final private static int BUFFER_SIZE = 2048;

   private LiveMeshDisplay display;

   private DataBuffer buffer;

   private YoVariableRegistry variableRegistry;

   private YoGraphicsListRegistry graphicsRegistry;

   private YoVariableClient yoVariableClient;

   private YoGraphicMeshProvider yoGraphicMeshProvider = new YoGraphicMeshProvider();

   private final ArrayList<JointUpdater> jointUpdaters = new ArrayList<>();

   @Override public void start(Stage stage) throws Exception
   {
      buffer = new DataBuffer(BUFFER_SIZE);

      stage.setScene(new Scene(display = new LiveMeshDisplay()));

      stage.show();

      new LiveMeshUpdater(display, yoGraphicMeshProvider).start();

      yoVariableClient = new YoVariableClient(this);
   }

   public static void main(String[] args) {
      launch(args);
   }

   @Override public void clearLog(String guid)
   {
      // TODO
   }

   @Override public void receivedTimestampOnly(long timestamp)
   {
      // TODO
   }

   @Override public boolean updateYoVariables()
   {
      return false;
   }

   @Override public boolean changesVariables()
   {
      return false;
   }

   @Override public void setShowOverheadView(boolean showOverheadView)
   {

   }

   @Override public void start(LogHandshake handshake, YoVariableHandshakeParser handshakeParser)
   {
      Robot robot = new Robot("DummyRobot");
      if (handshake.getModelLoaderClass() != null)
      {
         LogModelLoader modelLoader;
         try
         {
            modelLoader = (LogModelLoader) Class.forName(handshake.getModelLoaderClass()).newInstance();
         }
         catch (Exception e)
         {
            System.err.println("Could not instantiate LogModelLoader: " + handshake.getModelLoaderClass() + ". Defaulting to SDFModelLoader.");
            modelLoader = new SDFModelLoader();
         }
         modelLoader.load(handshake.getModelName(), handshake.getModel(), handshake.getResourceDirectories(), handshake.getResourceZip(), null);
         robot = new RobotFromDescription(modelLoader.createRobot());
      }

      this.variableRegistry = new YoVariableRegistry("default");

      YoVariableRegistry yoVariableRegistry = handshakeParser.getRootRegistry();
      this.variableRegistry.addChild(yoVariableRegistry);
      this.variableRegistry.addChild(yoVariableClient.getDebugRegistry());

      List<JointState> jointStates = handshakeParser.getJointStates();
      JointUpdater.getJointUpdaterList(robot.getRootJoints(), jointStates, jointUpdaters);

      graphicsRegistry = handshakeParser.getYoGraphicsListRegistry();
   }

   @Override public void disconnected()
   {

   }

   @Override public void setYoVariableClient(YoVariableClient client)
   {

   }

   @Override public int getDisplayOneInNPackets()
   {
      return 0;
   }

   @Override public void receivedTimestampAndData(long timestamp)
   {
      List<MeshView> meshes = new ArrayList<>();

      for (YoGraphicsList list : graphicsRegistry.getYoGraphicsLists()) {
         for (YoGraphic graphic : list.getYoGraphics()) {
            meshes.add((MeshView) new JavaFX3DInstructionExecutor(graphic.getLinkGraphics().getGraphics3DInstructions()).getResult());
         }
      }

      yoGraphicMeshProvider.setLater(meshes);
   }

   @Override public boolean executeVariableChangedListeners()
   {
      return false;
   }
}
