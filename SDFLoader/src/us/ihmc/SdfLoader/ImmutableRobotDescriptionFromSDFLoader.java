package us.ihmc.SdfLoader;

import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.Camera;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.IMU;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.IMU.IMUNoise;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.IMU.IMUNoise.NoiseParameters;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.Ray;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.Ray.Noise;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.Ray.Range;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.Ray.Scan;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.Ray.Scan.HorizontalScan;
import us.ihmc.SdfLoader.xmlDescription.SDFSensor.Ray.Scan.VerticalScan;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.robotics.geometry.InertiaTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.immutableRobotDescription.*;
import us.ihmc.robotics.immutableRobotDescription.OneDoFJointDescription.LimitStops;
import us.ihmc.robotics.immutableRobotDescription.graphics.GraphicsGroupDescription;
import us.ihmc.robotics.lidar.LidarScanParameters;
import us.ihmc.robotics.partNames.JointNameMap;
import us.ihmc.simulationconstructionset.simulatedSensors.SimulatedLIDARSensorLimitationParameters;
import us.ihmc.simulationconstructionset.simulatedSensors.SimulatedLIDARSensorNoiseParameters;
import us.ihmc.simulationconstructionset.simulatedSensors.SimulatedLIDARSensorUpdateParameters;

import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import javax.xml.bind.JAXBException;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.*;

import static java.lang.Double.parseDouble;

public class ImmutableRobotDescriptionFromSDFLoader
{
   // TODO: there should be methods to generated the graphics based on the link description, not be static defines burried in a loader that the user cannot change
   /*private static final boolean SHOW_CONTACT_POINTS = true;
   private static final boolean SHOW_COM_REFERENCE_FRAMES = false;
   private static final boolean SHOW_INERTIA_ELLIPSOIDS = false;
   private static final boolean SHOW_SENSOR_REFERENCE_FRAMES = false;*/

   public RobotDescription loadRobotDescriptionFromSDF(String modelName, InputStream inputStream, List<String> resourceDirectories, SDFDescriptionMutator mutator, JointNameMap sdfJointNameMap, boolean useCollisionMeshes,
         boolean enableTorqueVelocityLimits, boolean enableDamping)
   {
      GeneralizedSDFRobotModel generalizedSDFRobotModel = loadSDFFile(modelName, inputStream, resourceDirectories, mutator);
      return loadRobotDescriptionFromSDF(generalizedSDFRobotModel, sdfJointNameMap, useCollisionMeshes, enableTorqueVelocityLimits, enableDamping);
   }

   public RobotDescription loadRobotDescriptionFromSDF(GeneralizedSDFRobotModel generalizedSDFRobotModel, JointNameMap sdfJointNameMap, boolean useCollisionMeshes, boolean enableTorqueVelocityLimits, boolean enableDamping)
   {
      List<String> resourceDirectories = generalizedSDFRobotModel.getResourceDirectories();

      String name = generalizedSDFRobotModel.getName();
      ImmutableRobotDescription.Builder robotDescription = RobotDescription.builder().name(name);

      ArrayList<SDFLinkHolder> rootLinks = generalizedSDFRobotModel.getRootLinks();

      if (rootLinks.size() > 1)
      {
         throw new RuntimeException("Can only accommodate one root link for now");
      }

      SDFLinkHolder rootLink = rootLinks.get(0);

      Vector3d offset = new Vector3d();
      Quat4d orientation = new Quat4d();
      generalizedSDFRobotModel.getTransformToRoot().get(orientation, offset);
      ImmutableFloatingJointDescription.Builder rootJointDescription = FloatingJointDescription.builder().name(rootLink.getName());

      LinkDescription rootLinkDescription = createLinkDescription(rootLink, new RigidBodyTransform(), useCollisionMeshes, resourceDirectories);
      rootJointDescription.link(rootLinkDescription);
      ConvertedSensors sensors = convertSensors(rootLink);
      rootJointDescription.addAllCameraSensors(sensors.cameras);
      rootJointDescription.addAllIMUSensors(sensors.imus);
      rootJointDescription.addAllLidarSensors(sensors.lidars);

      // Ground Contact Points:

      Map<String, List<Vector3d>> sdfJointNameToGroundContactPoints = new HashMap<>();
      if (sdfJointNameMap != null)
      {
         groupContactPointsByJoint(sdfJointNameMap);

         enableTorqueVelocityLimits = enableTorqueVelocityLimits && sdfJointNameMap.isTorqueVelocityLimitsEnabled();
      }

      ConversionParameters conversionParameters = new ConversionParameters(useCollisionMeshes, enableTorqueVelocityLimits, enableDamping,
                                                                           sdfJointNameToGroundContactPoints, sdfJointNameMap, resourceDirectories);

      for (SDFJointHolder child : rootLink.getChildren())
      {
         JointDescription convertedChildJoint = convertJointsRecursively(child, false, conversionParameters);
         rootJointDescription.addChildrenJoints(convertedChildJoint);
      }

      robotDescription.addChildrenJoints(rootJointDescription.build());

      return robotDescription.build();
   }

   private Map<String, List<Vector3d>> groupContactPointsByJoint(JointNameMap sdfJointNameMap)
   {
      Map<String, List<Vector3d>> result = new HashMap<>();
      if (sdfJointNameMap == null)
         return result;

      for (ImmutablePair<String, Vector3d> jointContactPoint : sdfJointNameMap.getJointNameGroundContactPointMap())
      {
         List<Vector3d> contactPoints = result.get(jointContactPoint.getLeft());
         if (contactPoints == null)
            contactPoints = new ArrayList<>();
         contactPoints.add(jointContactPoint.getRight());
         result.put(jointContactPoint.getLeft(), contactPoints);
      }
      return result;
   }

   // FIXME: graphics should be immutable as well
   /*private void convertLinkGraphics(JointDescription jointDescription) {
      if (SHOW_CONTACT_POINTS)
      {
         for (GroundContactPointDescription groundContactPointDescription : jointDescription.getGroundContactPoints())
         {
            GraphicsGroupDescription graphics = jointDescription.getLink().getLinkGraphics();

            graphics.identity();
            graphics.translate(groundContactPointDescription.getOffsetFromJoint());
            double radius = 0.01;
            graphics.addSphere(radius, YoAppearance.Orange());
         }
      }

      List<SensorDescription> sensors = new ArrayList<>();
      sensors.addAll(jointDescription.getCameraSensors());
      sensors.addAll(jointDescription.getIMUSensors());
      sensors.addAll(jointDescription.getLidarSensors());
      for (SensorDescription sensor : sensors)
      {
         showCordinateSystem(jointDescription, sensor.getTransformToJoint());
      }
   }*/

   private List<JointContactPoints> convertGroundContactPoints(SDFJointHolder joint, List<Vector3d> groundContactPoints) {

      List<JointContactPoints> result = new ArrayList<>();
      String sanitizedJointName = SDFConversionsHelper.sanitizeJointName(joint.getName());
      for (int i = 0; i < groundContactPoints.size(); i++)
      {
         Vector3d gcOffset = groundContactPoints.get(i);
         GroundContactPointDescription groundContactPoint = GroundContactPointDescription.builder()
               .name("gc_" + sanitizedJointName + "_" + i)
               .offsetFromJoint(gcOffset)
               .build();
         ExternalForcePointDescription externalForcePoint = ExternalForcePointDescription.builder()
               .name("ef_" + sanitizedJointName + "_")
               .offsetFromJoint(gcOffset)
               .build();

         result.add(new JointContactPoints(groundContactPoint, externalForcePoint));
      }

      return result;
   }

   private static class JointContactPoints {
      final GroundContactPointDescription groundContactPointDescription;
      final ExternalForcePointDescription externalForcePointDescription;

      private JointContactPoints(GroundContactPointDescription groundContactPointDescription, ExternalForcePointDescription externalForcePointDescription)
      {
         this.groundContactPointDescription = groundContactPointDescription;
         this.externalForcePointDescription = externalForcePointDescription;
      }
   }

   private LinkDescription createLinkDescription(SDFLinkHolder link, RigidBodyTransform rotationTransform, boolean useCollisionMeshes, List<String> resourceDirectories)
   {
      ImmutableLinkDescription.Builder scsLinkBuilder = LinkDescription.builder().name(link.getName());

      //TODO: Get collision meshes working.
      if (useCollisionMeshes)
      {
         Graphics3DObject linkGraphicsDescription = new SDFGraphics3DObjectImmutable(link.getCollisions(), resourceDirectories, rotationTransform);

         scsLinkBuilder.linkGraphics(GraphicsGroupDescription.fromGraphics3DObject(linkGraphicsDescription));
      }
      else if (link.getVisuals() != null)
      {
         Graphics3DObject linkGraphicsDescription = new SDFGraphics3DObjectImmutable(link.getVisuals(), resourceDirectories, rotationTransform);
         scsLinkBuilder.linkGraphics(GraphicsGroupDescription.fromGraphics3DObject(linkGraphicsDescription));
      }

      double mass = link.getMass();
      Matrix3d inertia = InertiaTools.rotate(rotationTransform, link.getInertia());
      Vector3d CoMOffset = new Vector3d(link.getCoMOffset());

      if (link.getJoint() != null && isJointInNeedOfReducedGains(link.getJoint()))
      {
         inertia.mul(100.0);
      }

      rotationTransform.transform(CoMOffset);

      scsLinkBuilder.centerOfMassOffset(CoMOffset);
      scsLinkBuilder.mass(mass);
      scsLinkBuilder.momentOfInertia(LinkDescription.convertMomentOfInertia(inertia));

      return scsLinkBuilder.build();
   }

   private JointDescription convertJointsRecursively(SDFJointHolder joint, boolean doNotSimulateJoint, ConversionParameters parameters)
   {
      final Set<String> lastSimulatedJoints = parameters.sdfJointNameMap != null ? parameters.sdfJointNameMap.getLastSimulatedJoints() : new HashSet<String>();

      Vector3d jointAxis = new Vector3d(joint.getAxisInModelFrame());
      Vector3d offset = new Vector3d(joint.getOffsetFromParentJoint());

      RigidBodyTransform visualTransform = new RigidBodyTransform();
      visualTransform.setRotation(joint.getLinkRotation());

      String sanitizedJointName = SDFConversionsHelper.sanitizeJointName(joint.getName());

      OneDoFJointDescription.Builder scsJointBuilder;


      switch (joint.getType())
      {
      case REVOLUTE:
         ImmutablePinJointDescription.Builder pinJoint = PinJointDescription.builder().name(sanitizedJointName).offsetFromJoint(offset).axis(jointAxis);

         if (joint.hasLimits())
         {
            boolean reduceGains = isJointInNeedOfReducedGains(joint);
            double defaultContactKp = reduceGains ? 10.0 : 100.0; // TODO: why these constants?
            double defaultContactKd = reduceGains ? 2.5 : 20.0; // TODO: why these constants?
            double contactKp = joint.getContactKp() == 0 ? defaultContactKp : 0.0001 * joint.getContactKp(); // TODO: why these constants?
            double contactKd = joint.getContactKd() == 0 ? defaultContactKd : 0.1 * joint.getContactKd(); // TODO: why these constants?
            pinJoint.limitStops(new LimitStops(joint.getLowerLimit(), joint.getUpperLimit(), contactKp, contactKd));

            if (reduceGains && !Double.isNaN(joint.getVelocityLimit()))
            {
               pinJoint.velocityLimit(joint.getVelocityLimit())
                       .velocityDamping(0.0);
            }
         }

         if (parameters.enableTorqueVelocityLimits && !isJointInNeedOfReducedGains(joint))
         {
            if (!Double.isNaN(joint.getEffortLimit()))
            {
               pinJoint.effortLimit(joint.getEffortLimit());
            }

            if (!Double.isNaN(joint.getVelocityLimit()))
            {
               pinJoint.velocityLimit(joint.getVelocityLimit())
                       .velocityDamping(500.0); // TODO: why 500?
            }
         }

         scsJointBuilder = pinJoint;

         break;

      case PRISMATIC:
         ImmutableSliderJointDescription.Builder sliderJoint = SliderJointDescription.builder()
               .name(sanitizedJointName)
               .offsetFromJoint(offset)
               .axis(jointAxis);
         if (joint.hasLimits())
         {
            double contactKp = joint.getContactKp() == 0 ? 100.0 : 0.0001 * joint.getContactKp(); // TODO: why these constants?
            double contactKd = joint.getContactKd() == 0 ? 20.0 : joint.getContactKd(); // TODO: why these constants?
            sliderJoint.limitStops(new LimitStops(joint.getLowerLimit(), joint.getUpperLimit(), contactKp, contactKd));
         }

         scsJointBuilder = sliderJoint;
         break;

      default:
         throw new RuntimeException("Joint type not implemented: " + joint.getType());
      }

      if (parameters.enableDamping)
      {
         scsJointBuilder.damping(joint.getDamping());
         scsJointBuilder.stiction(joint.getFriction());
      }

      if (doNotSimulateJoint) scsJointBuilder.dynamic(false);

      scsJointBuilder.link(createLinkDescription(joint.getChildLinkHolder(), visualTransform, parameters.useCollisionMeshes, parameters.resourceDirectories));

      ConvertedSensors convertedSensors = convertSensors(joint.getChildLinkHolder());
      scsJointBuilder.addAllCameraSensors(convertedSensors.cameras);
      scsJointBuilder.addAllIMUSensors(convertedSensors.imus);
      scsJointBuilder.addAllLidarSensors(convertedSensors.lidars);

      List<Vector3d> groundContacts = parameters.sdfJointNameToGroundContactPoint.get(sanitizedJointName);
      if (groundContacts == null)
         groundContacts = Collections.emptyList();
      List<JointContactPoints> groundContactPoints = convertGroundContactPoints(joint, groundContacts);
      for (JointContactPoints groundContactPoint : groundContactPoints)
      {
         scsJointBuilder.addGroundContactPoints(groundContactPoint.groundContactPointDescription);
         scsJointBuilder.addExternalForcePoints(groundContactPoint.externalForcePointDescription);
      }


      List<ForceSensorDescription> forceSensors = convertForceSensors(joint, parameters.sdfJointNameMap);
      scsJointBuilder.addAllForceSensors(forceSensors);

      if (!doNotSimulateJoint && lastSimulatedJoints.contains(joint.getName()))
      {
         doNotSimulateJoint = true;
      }

      for (SDFJointHolder child : joint.getChildLinkHolder().getChildren())
      {
         JointDescription childJoint = convertJointsRecursively(child, doNotSimulateJoint, parameters);
         scsJointBuilder.addChildrenJoints(childJoint);
      }
      return scsJointBuilder.build();
   }

   ///TODO:
   ///XXX: pull these names from the sdfJointNameMap
   private boolean isJointInNeedOfReducedGains(SDFJointHolder pinJoint)
   {
      String jointName = pinJoint.getName();
      return jointName.contains("f0") || jointName.contains("f1") || jointName.contains("f2") || jointName.contains("f3") || jointName.contains("palm") || jointName.contains("finger");
   }

   private static GeneralizedSDFRobotModel loadSDFFile(String modelName, InputStream inputStream, List<String> resourceDirectories, SDFDescriptionMutator mutator)
   {
      GeneralizedSDFRobotModel generalizedSDFRobotModel;
      try
      {
         JaxbSDFLoader loader = new JaxbSDFLoader(inputStream, resourceDirectories, mutator);
         generalizedSDFRobotModel = loader.getGeneralizedSDFRobotModel(modelName);
      }
      catch (FileNotFoundException | JAXBException e)
      {
         throw new RuntimeException("Cannot load model", e);
      }

      return generalizedSDFRobotModel;
   }

   /*private void showCordinateSystem(JointDescription scsJoint, RigidBodyTransform offsetFromLink)
   {
      if (SHOW_SENSOR_REFERENCE_FRAMES)
      {
         Graphics3DObject linkGraphics = scsJoint.getLink().getLinkGraphics();
         linkGraphics.identity();
         linkGraphics.transform(offsetFromLink);
         linkGraphics.addCoordinateSystem(1.0);
         linkGraphics.identity();
      }
   }*/

   private ConvertedSensors convertSensors(SDFLinkHolder child)
   {
      ConvertedSensors result = new ConvertedSensors();
      if (child.getSensors() == null)
         return result;

      for (SDFSensor sensor : child.getSensors())
      {
         switch (sensor.getType())
         {
         case "camera":
         case "multicamera":
            result.cameras.addAll(convertCameraMounts(sensor, child));
            break;
         case "imu":
            result.imus.addAll(convertIMUMounts(sensor, child));
            break;
         case "gpu_ray":
         case "ray":
            result.lidars.addAll(convertLidarMounts(sensor, child));
            break;
         }
      }
      return result;
   }

   private List<CameraSensorDescription> convertCameraMounts(SDFSensor sensor, SDFLinkHolder child)
   {
      // TODO: handle left and right sides of multicamera
      final List<Camera> cameras = sensor.getCamera();

      List<CameraSensorDescription> result = new ArrayList<>();

      if (cameras == null) {
         System.err.println("JAXB loader: No camera section defined for camera sensor " + sensor.getName() + ", ignoring sensor.");
         return result;
      }

      for (Camera camera : cameras)
      {
         // The linkRotation transform is to make sure that the linkToSensor is in a zUpFrame.
         RigidBodyTransform linkRotation = new RigidBodyTransform(child.getTransformFromModelReferenceFrame());
         linkRotation.setTranslation(0.0, 0.0, 0.0);
         RigidBodyTransform linkToSensor = SDFConversionsHelper.poseToTransform(sensor.getPose());
         RigidBodyTransform sensorToCamera = SDFConversionsHelper.poseToTransform(camera.getPose());
         RigidBodyTransform linkToCamera = new RigidBodyTransform();
         linkToCamera.multiply(linkRotation, linkToSensor);
         linkToCamera.multiply(sensorToCamera);

         double fieldOfView = parseDouble(camera.getHorizontalFov());
         double clipNear = parseDouble(camera.getClip().getNear());
         double clipFar = parseDouble(camera.getClip().getFar());
         String cameraName = sensor.getName() + "_" + camera.getName();
         ImmutableCameraSensorDescription.Builder mount = CameraSensorDescription.builder()
               .name(cameraName)
               .transformToJoint(linkToCamera)
               .fieldOfView(fieldOfView)
               .clipNear(clipNear)
               .clipFar(clipFar);

         int imageHeight = Integer.parseInt(camera.getImage().getHeight());
         int imageWidth = Integer.parseInt(camera.getImage().getWidth());

         mount.imageHeight(imageHeight);
         mount.imageWidth(imageWidth);

         result.add(mount.build());
      }

      return result;
   }

   private List<IMUSensorDescription> convertIMUMounts(SDFSensor sdfSensor, SDFLinkHolder child)
   {
      // TODO: handle left and right sides of multicamera
      final IMU imu = sdfSensor.getImu();
      if (imu == null) {
         System.err.println("JAXB loader: No imu section defined for imu sensor " + sdfSensor.getName() + ", ignoring sensor.");
         return Collections.emptyList();
      }

      // The linkRotation transform is to make sure that the linkToSensor is in a zUpFrame.
      RigidBodyTransform linkRotation = new RigidBodyTransform(child.getTransformFromModelReferenceFrame());
      linkRotation.setTranslation(0.0, 0.0, 0.0);
      RigidBodyTransform linkToSensorInZUp = new RigidBodyTransform();
      linkToSensorInZUp.multiply(linkRotation, SDFConversionsHelper.poseToTransform(sdfSensor.getPose()));

      ImmutableIMUSensorDescription.Builder imuMount = IMUSensorDescription.builder()
            .name(child.getName() + "_" + sdfSensor.getName())
            .transformToJoint(linkToSensorInZUp);

      IMUNoise noise = imu.getNoise();
      if (noise != null)
      {
         if ("gaussian".equals(noise.getType()))
         {
            NoiseParameters accelerationNoise = noise.getAccel();
            NoiseParameters angularVelocityNoise = noise.getRate();

            GaussianParameter outAccelNoise = GaussianParameter.fromMeanStd(parseDouble(accelerationNoise.getMean()), parseDouble(accelerationNoise.getStddev()));
            GaussianParameter outAccelBias = GaussianParameter.fromMeanStd(parseDouble(accelerationNoise.getBias_mean()), parseDouble(accelerationNoise.getBias_stddev()));

            GaussianParameter outAngularVelNoise = GaussianParameter.fromMeanStd(parseDouble(angularVelocityNoise.getMean()), parseDouble(angularVelocityNoise.getStddev()));
            GaussianParameter outAngularValBias = GaussianParameter.fromMeanStd(parseDouble(angularVelocityNoise.getBias_mean()), parseDouble(angularVelocityNoise.getBias_stddev()));

            imuMount.accelerationNoise(outAccelNoise)
                  .accelerationBias(outAccelBias)
                  .angularVelocityNoise(outAngularVelNoise)
                  .angularVelocityBias(outAngularValBias);
         }
         else
         {
            throw new RuntimeException("Unknown IMU noise model: " + noise.getType());
         }
      }

      return Collections.singletonList((IMUSensorDescription)imuMount.build());
   }

   private List<LidarSensorDescription> convertLidarMounts(SDFSensor sensor, SDFLinkHolder child)
   {
      List<LidarSensorDescription> result = new ArrayList<>();
      Ray sdfRay = sensor.getRay();
      if (sdfRay == null)
      {
         System.err.println("SDFRobot: lidar not present in ray type sensor " + sensor.getName() + ". Ignoring this sensor.");
         return result;
      }

      Range sdfRange = sdfRay.getRange();
      Scan sdfScan = sdfRay.getScan();
      double sdfMaxRange = parseDouble(sdfRange.getMax());
      double sdfMinRange = parseDouble(sdfRange.getMin());
      HorizontalScan sdfHorizontalScan = sdfScan.getHorizontal();
      VerticalScan sdfVerticalScan = sdfScan.getVertical();
      double sdfMaxSweepAngle = parseDouble(sdfHorizontalScan.getMaxAngle());
      double sdfMinSweepAngle = parseDouble(sdfHorizontalScan.getMinAngle());
      double sdfMaxHeightAngle = sdfVerticalScan == null ? 0.0 : parseDouble(sdfVerticalScan.getMaxAngle());
      double sdfMinHeightAngle = sdfVerticalScan == null ? 0.0 : parseDouble(sdfVerticalScan.getMinAngle());

      // double sdfAngularResolution = Double.parseDouble(sdfHorizontalScan.getSillyAndProbablyNotUsefulResolution());
      int sdfSamples = (Integer.parseInt(sdfHorizontalScan.getSamples()) / 3) * 3;
      int sdfScanHeight = sdfVerticalScan == null ? 1 : Integer.parseInt(sdfVerticalScan.getSamples());
      double sdfRangeResolution = parseDouble(sdfRay.getRange().getResolution());

      final boolean sdfAlwaysOn = true;

      double sdfGaussianStdDev = 0.0;
      double sdfGaussianMean = 0.0;
      int sdfUpdateRate = (int) (1000.0 / parseDouble(sensor.getUpdateRate()));

      Noise sdfNoise = sdfRay.getNoise();
      if (sdfNoise != null)
      {
         if ("gaussian".equals(sdfNoise.getType()))
         {
            sdfGaussianStdDev = parseDouble(sdfNoise.getStddev());
            sdfGaussianMean = parseDouble(sdfNoise.getMean());
         }
         else
         {
            System.err.println("Unknown noise model: " + sdfNoise.getType());
         }
      }

      //         System.err.println("[SDFRobot]: FIXME: Setting LIDAR angle to 0.5 pi due to current GPULidar limitations");
      //         sdfMinAngle = -Math.PI/4;
      //         sdfMaxAngle = Math.PI/4;

      LidarScanParameters polarDefinition = new LidarScanParameters(sdfSamples, sdfScanHeight, (float) sdfMinSweepAngle, (float) sdfMaxSweepAngle, (float) sdfMinHeightAngle, (float) sdfMaxHeightAngle, 0.0f,
                                                                    (float) sdfMinRange, (float) sdfMaxRange, 0.0f, 0L);

      // The linkRotation transform is to make sure that the linkToSensor is in a zUpFrame.
      RigidBodyTransform linkRotation = new RigidBodyTransform(child.getTransformFromModelReferenceFrame());
      linkRotation.setTranslation(0.0, 0.0, 0.0);
      RigidBodyTransform linkToSensorInZUp = new RigidBodyTransform();
      linkToSensorInZUp.multiply(linkRotation, SDFConversionsHelper.poseToTransform(sensor.getPose()));

      SimulatedLIDARSensorNoiseParameters noiseParameters = new SimulatedLIDARSensorNoiseParameters();
      noiseParameters.setGaussianNoiseStandardDeviation(sdfGaussianStdDev);
      noiseParameters.setGaussianNoiseMean(sdfGaussianMean);

      SimulatedLIDARSensorLimitationParameters limitationParameters = new SimulatedLIDARSensorLimitationParameters();
      limitationParameters.setMaxRange(sdfMaxRange);
      limitationParameters.setMinRange(sdfMinRange);
      limitationParameters.setQuantization(sdfRangeResolution);

      SimulatedLIDARSensorUpdateParameters updateParameters = new SimulatedLIDARSensorUpdateParameters();
      updateParameters.setAlwaysOn(sdfAlwaysOn);
      updateParameters.setUpdatePeriodInMillis(sdfUpdateRate);

      LidarSensorDescription lidarMount = LidarSensorDescription.builder()
            .name(sensor.getName())
            .transformToJoint(linkToSensorInZUp)
            .lidarScanParameters(polarDefinition)
            .build();
      result.add(lidarMount);
      return result;
   }

   private List<ForceSensorDescription> convertForceSensors(SDFJointHolder joint, JointNameMap jointNameMap)
   {
      List<ForceSensorDescription> result = new ArrayList<>();
      if (joint.getForceSensors().isEmpty())
         return result;

      String[] jointNamesBeforeFeet = jointNameMap.getJointNamesBeforeFeet();

      String jointName = joint.getName();

      boolean jointIsParentOfFoot = false;
      for (String aJointNamesBeforeFeet : jointNamesBeforeFeet)
      {
         if (jointName.equals(aJointNamesBeforeFeet))
         {
            jointIsParentOfFoot = true;
            break;
         }
      }

      for (SDFForceSensor forceSensor : joint.getForceSensors())
      {
         ImmutableForceSensorDescription.Builder forceSensorDescription = ForceSensorDescription.builder().name(forceSensor.getName()).transformToJoint(forceSensor.getTransform());
         forceSensorDescription.useGroundContactPoints(jointIsParentOfFoot);
         result.add(forceSensorDescription.build());
      }
      return result;
   }

   private static class ConversionParameters {
      final boolean useCollisionMeshes;
      final boolean enableTorqueVelocityLimits;
      final boolean enableDamping;
      final Map<String, List<Vector3d>> sdfJointNameToGroundContactPoint;
      final JointNameMap sdfJointNameMap;
      final List<String> resourceDirectories;

      private ConversionParameters(boolean useCollisionMeshes, boolean enableTorqueVelocityLimits, boolean enableDamping,
                                   Map<String, List<Vector3d>> sdfJointNameToGroundContactPoint, JointNameMap sdfJointNameMap,
                                   List<String> resourceDirectories)
      {
         this.useCollisionMeshes = useCollisionMeshes;
         this.enableTorqueVelocityLimits = enableTorqueVelocityLimits;
         this.enableDamping = enableDamping;
         this.sdfJointNameToGroundContactPoint = sdfJointNameToGroundContactPoint;
         this.sdfJointNameMap = sdfJointNameMap;
         this.resourceDirectories = resourceDirectories;
      }
   }

   private static class ConvertedSensors {
      final List<CameraSensorDescription> cameras = new ArrayList<>();
      final List<IMUSensorDescription> imus = new ArrayList<>();
      final List<LidarSensorDescription> lidars = new ArrayList<>();
   }
}
