/**
 * 
 */
package us.ihmc.exampleSimulations.stickRobot;

import java.util.LinkedHashMap;

import org.apache.commons.lang3.tuple.ImmutablePair;

import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;

import java.io.FileInputStream;
import java.io.InputStream;
import java.io.FileNotFoundException;

import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.avatar.handControl.HandCommandManager;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.time.DRCROSAlwaysZeroOffsetPPSTimestampOffsetProvider;
import us.ihmc.avatar.ros.DRCROSPPSTimestampOffsetProvider;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;
import us.ihmc.robotModels.FullHumanoidRobotModelFromDescription;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.humanoidRobotics.footstep.footstepGenerator.FootstepPlanningParameterization;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.FootstepSnappingParameters;
import us.ihmc.modelFileLoaders.SdfLoader.DRCRobotSDFLoader;
import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.modelFileLoaders.SdfLoader.JaxbSDFLoader;
import us.ihmc.modelFileLoaders.SdfLoader.SDFContactSensor;
import us.ihmc.modelFileLoaders.SdfLoader.SDFDescriptionMutator;
import us.ihmc.modelFileLoaders.SdfLoader.SDFForceSensor;
import us.ihmc.modelFileLoaders.SdfLoader.SDFJointHolder;
import us.ihmc.modelFileLoaders.SdfLoader.SDFLinkHolder;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFSensor;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.SDFLogModelProvider;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.robotController.OutputProcessor;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.robotController.MultiThreadedRobotControlElement;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.wholeBodyController.DRCHandType;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.FootContactPoints;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;
import us.ihmc.wholeBodyController.parameters.DefaultArmConfigurations;
import us.ihmc.modelFileLoaders.SdfLoader.RobotDescriptionFromSDFLoader;

/**
 * @author Shlok Agarwal on 7/5/17
 *
 */
public class StickRobotModel implements DRCRobotModel, SDFDescriptionMutator
{

   private static final boolean PRINT_MODEL = false;
   
   private final CapturePointPlannerParameters capturePointPlannerParameters;
   private final WalkingControllerParameters walkingControllerParameters;
   private final StateEstimatorParameters stateEstimatorParamaters;
   private final DRCRobotPhysicalProperties physicalProperties;
   private final StickRobotJointMap jointMap;
   private final StickRobotContactPointParameters contactPointParameters;
   private final String robotName = "STICK_BOT";
   private final SideDependentList<Transform> offsetHandFromWrist = new SideDependentList<Transform>();
   
   private final DRCRobotModel.RobotTarget target;

   private final String[] resourceDirectories;
   {
         resourceDirectories = new String[]{
               "models/stickRobot/sdf"
            };
   }

   private final JaxbSDFLoader loader;
   private final RobotDescription robotDescription;
   
   private boolean enableJointDamping = true;
   
   
   // One of the three constructors below calls the fourth constructor with different configuration settings
   public StickRobotModel(DRCRobotModel.RobotTarget target, boolean headless, FootContactPoints simulationContactPoints)
   {
      this(target,headless, "DEFAULT", simulationContactPoints);
   }

   public StickRobotModel(DRCRobotModel.RobotTarget target, boolean headless)
   {
      this(target,headless, "DEFAULT", null);
   }

   public StickRobotModel(DRCRobotModel.RobotTarget target, boolean headless, String model)
   {
      this(target, headless, model, null);
   }

   // main constructor being called 
   public StickRobotModel(DRCRobotModel.RobotTarget target, boolean headless, String model, FootContactPoints simulationContactPoints)
   {
      this.target = target;
      jointMap = new StickRobotJointMap();
      contactPointParameters = new StickRobotContactPointParameters(jointMap, simulationContactPoints);
      physicalProperties = new StickRobotPhysicalProperties();
      InputStream sdf = null;

      if(model.equalsIgnoreCase("DEFAULT"))
      {
         System.out.println("Loading robot model from: '"+getSdfFile()+"'");
         sdf=getSdfFileAsStream();
      }
      else
      {
         System.out.println("Loading robot model from: '"+model+"'");
         sdf=getClass().getClassLoader().getResourceAsStream(model);
         if(sdf==null)
         {
            try
            {
               sdf=new FileInputStream(model);
            }
            catch (FileNotFoundException e)
            {
               System.err.println("failed to load sdf file - file not found");
            }
         }

      }

      this.loader = DRCRobotSDFLoader.loadDRCRobot(getResourceDirectories(), sdf, this);

      boolean runningOnRealRobot = target == RobotTarget.REAL_ROBOT;
      capturePointPlannerParameters = null;
      walkingControllerParameters = null;
      stateEstimatorParamaters = null;
      robotDescription = createRobotDescription();
   }

   private RobotDescription createRobotDescription()
   {
      boolean useCollisionMeshes = false;

      GeneralizedSDFRobotModel generalizedSDFRobotModel = getGeneralizedRobotModel();
      RobotDescriptionFromSDFLoader descriptionLoader = new RobotDescriptionFromSDFLoader();
      RobotDescription robotDescription = descriptionLoader.loadRobotDescriptionFromSDF(generalizedSDFRobotModel, jointMap, contactPointParameters, useCollisionMeshes);
      return robotDescription;
   }


   public GeneralizedSDFRobotModel getGeneralizedRobotModel()
   {
      return loader.getGeneralizedSDFRobotModel(getJointMap().getModelName());
   }
   
   private InputStream getSdfFileAsStream()
   {
      return getClass().getClassLoader().getResourceAsStream(getSdfFile());
   }
   
   @Override
   public String toString()
   {
      return robotName;
   }
   
   private String getSdfFile()
   {
      return "models/stickRobot/sdf/stickbot_humanoid_32dof.sdf";
//      if(this.target == RobotTarget.REAL_ROBOT)
//         return ValkyrieConfigurationRoot.REAL_ROBOT_SDF_FILE;
//      else
//         return ValkyrieConfigurationRoot.SIM_SDF_FILE;
   }

   private String[] getResourceDirectories()
   {
      return resourceDirectories;
   }

   
   @Override
   public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes)
   {
      boolean enableTorqueVelocityLimits = false;
      boolean enableJointDamping = getEnableJointDamping();

      HumanoidFloatingRootJointRobot sdfRobot = new HumanoidFloatingRootJointRobot(robotDescription, jointMap, enableJointDamping, enableTorqueVelocityLimits);

      if (PRINT_MODEL)
      {
         System.out.println("\nStickRobotModel Link Masses:");

         StringBuffer stringBuffer = new StringBuffer();
         sdfRobot.printRobotJointsAndMasses(stringBuffer);
         System.out.println(stringBuffer);
         System.out.println();

         System.out.println("nStickRobotModel: \n" + sdfRobot);
      }

      return sdfRobot;
   }

   @Override
   public CapturePointPlannerParameters getCapturePointPlannerParameters()
   {
      return capturePointPlannerParameters;
   }

   @Override
   public ICPOptimizationParameters getICPOptimizationParameters()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public WalkingControllerParameters getWalkingControllerParameters()
   {
      return walkingControllerParameters;
   }

   @Override
   public RobotContactPointParameters getContactPointParameters()
   {
      return contactPointParameters;
   }

   @Override
   public double getControllerDT()
   {
      return 0.004;
   }

   @Override
   public OutputProcessor getOutputProcessor(FullRobotModel controllerFullRobotModel)
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public DefaultArmConfigurations getDefaultArmConfigurations()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public DRCRobotSensorInformation getSensorInformation()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public FullHumanoidRobotModel createFullRobotModel()
   {    
      return new FullHumanoidRobotModelFromDescription(robotDescription, jointMap);
   }

   @Override
   public RobotDescription getRobotDescription()
   {
      return robotDescription;
   }

   @Override
   public void mutateJointForModel(GeneralizedSDFRobotModel model, SDFJointHolder jointHolder)
   {

      
   }

   @Override
   public void mutateLinkForModel(GeneralizedSDFRobotModel model, SDFLinkHolder linkHolder)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void mutateSensorForModel(GeneralizedSDFRobotModel model, SDFSensor sensor)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void mutateForceSensorForModel(GeneralizedSDFRobotModel model, SDFForceSensor forceSensor)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void mutateContactSensorForModel(GeneralizedSDFRobotModel model, SDFContactSensor contactSensor)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void mutateModelWithAdditions(GeneralizedSDFRobotModel model)
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public FootstepPlanningParameterization getFootstepParameters()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public StateEstimatorParameters getStateEstimatorParameters()
   {
      return stateEstimatorParamaters;
   }

   @Override
   public DRCRobotPhysicalProperties getPhysicalProperties()
   {
      return physicalProperties;
   }

   @Override
   public DRCRobotJointMap getJointMap()
   {
      return jointMap;
   }

   @Override
   public DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw)
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public void setEnableJointDamping(boolean enableJointDamping)
   {
      this.enableJointDamping   = enableJointDamping;
   }

   @Override
   public boolean getEnableJointDamping()
   {
      // TODO Auto-generated method stub
      return enableJointDamping;
   }

   @Override
   public HandModel getHandModel()
   {
      // TODO Auto-generated method stub
      // it is present for valkyrie
      return null;
   }

   @Override
   public Transform getJmeTransformWristToHand(RobotSide side)
   {
      createTransforms();
      return offsetHandFromWrist.get(side);
   }

   private void createTransforms()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         Vector3f centerOfHandToWristTranslation = new Vector3f();
         float[] angles = new float[3];

         centerOfHandToWristTranslation = new Vector3f(0f, (float) robotSide.negateIfLeftSide(0.015f), -0.06f);
         angles[0] = (float) robotSide.negateIfLeftSide(Math.toRadians(90));
         angles[1] = 0.0f;
         angles[2] = (float) robotSide.negateIfLeftSide(Math.toRadians(90));
         //
         Quaternion centerOfHandToWristRotation = new Quaternion(angles);
         offsetHandFromWrist.set(robotSide, new Transform(centerOfHandToWristTranslation, centerOfHandToWristRotation));
      }
   }
   @Override
   public RigidBodyTransform getTransform3dWristToHand(RobotSide side)
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public double getSimulateDT()
   {
      return 0.0001; //0.00003875;
   }

   @Override
   public double getEstimatorDT()
   {
      return 0.002;
   }

   @Override
   public double getStandPrepAngle(String jointName)
   {
      // TODO Auto-generated method stub
      return 0;
   }

   @Override
   public DRCROSPPSTimestampOffsetProvider getPPSTimestampOffsetProvider()
   {
      return new DRCROSAlwaysZeroOffsetPPSTimestampOffsetProvider();
   }

   @Override
   public DRCSensorSuiteManager getSensorSuiteManager()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public SideDependentList<HandCommandManager> createHandCommandManager()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public MultiThreadedRobotControlElement createSimulatedHandController(FloatingRootJointRobot simulatedRobot,
                                                                         ThreadDataSynchronizerInterface threadDataSynchronizer,
                                                                         HumanoidGlobalDataProducer globalDataProducer,
                                                                         CloseableAndDisposableRegistry closeableAndDisposableRegistry)
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public DRCHandType getDRCHandType()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public LinkedHashMap<NeckJointName, ImmutablePair<Double, Double>> getSliderBoardControlledNeckJointsWithLimits()
   {
      return walkingControllerParameters.getSliderBoardControlledNeckJointsWithLimits();
   }

   @Override
   public SideDependentList<LinkedHashMap<String, ImmutablePair<Double, Double>>> getSliderBoardControlledFingerJointsWithLimits()
   {
      return walkingControllerParameters.getSliderBoardControlledFingerJointsWithLimits();
   }

   @Override
   public LogSettings getLogSettings()
   {
      return LogSettings.SIMULATION;
   }

   @Override
   public LogModelProvider getLogModelProvider()
   {
      return new SDFLogModelProvider(jointMap.getModelName(), getSdfFileAsStream(), getResourceDirectories());
   }

   @Override
   public String getSimpleRobotName()
   {
      // TODO Auto-generated method stub
      return "StickRobot";
   }

   @Override
   public CollisionBoxProvider getCollisionBoxProvider()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public FootstepSnappingParameters getSnappingParameters()
   {
      // TODO Auto-generated method stub
      return null;
   }
   
}
