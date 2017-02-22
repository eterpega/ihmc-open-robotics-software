package us.ihmc.modelFileLoaders.urdfLoader;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.modelFileLoaders.ModelFileLoaderConversionsHelper;
import us.ihmc.modelFileLoaders.urdfLoader.xmlDescription.*;
import us.ihmc.modelFileLoaders.urdfLoader.xmlDescription.extensions.URDFGazebo;
import us.ihmc.robotics.partNames.JointNameMap;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.tools.io.printing.PrintTools;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBElement;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Unmarshaller;
import javax.xml.stream.XMLInputFactory;
import javax.xml.stream.XMLStreamConstants;
import javax.xml.stream.XMLStreamException;
import javax.xml.stream.XMLStreamReader;
import javax.xml.transform.stream.StreamSource;
import java.io.IOException;
import java.net.URL;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class RobotDescriptionFromURDFLoader
{
   private final JAXBContext coreContext;
   private final Unmarshaller coreUnmashaller;
   private String modelName;
   private List<String> resourceDirectories;

   public RobotDescriptionFromURDFLoader()
   {
      try
      {
         coreContext = JAXBContext.newInstance("us.ihmc.modelFileLoaders.urdfLoader.xmlDescription");
         coreUnmashaller = coreContext.createUnmarshaller();
      }
      catch (JAXBException e)
      {
         throw new RuntimeException(e);
      }
   }

   public RobotDescription loadRobotDescriptionFromURDF(String modelName, URL modelFileResourceURL, List<String> resourceDirectories, JointNameMap jointNameMap,
                                                        boolean useCollisionMeshes)
   {
      RobotDescription robotDescription;
      try
      {
         robotDescription = new RobotDescription(modelName);
         URDFRobot urdfRobot = (URDFRobot) coreUnmashaller.unmarshal(modelFileResourceURL);
         List<URDFGazebo> gazeboTags = getGazeboExtensions(modelFileResourceURL);

         HashMap<String, URDFJoint> allJointsFromURDFRobot = new HashMap<>();
         HashMap<String, URDFLink> allLinksFromURDFRobot = new HashMap<>();
         HashMap<String, URDFMaterialGlobal> allGlobalMaterialsFromURDFRobot = new HashMap<>();
         HashMap<String, URDFTransmission> allTransmissionFromURDFRobot = new HashMap<>();

         ModelFileLoaderConversionsHelper.getGlobalLinksJointsMaterialsAndTransmissionFromURDFRobot(urdfRobot, allJointsFromURDFRobot, allLinksFromURDFRobot, allGlobalMaterialsFromURDFRobot,
                                                                                                    allTransmissionFromURDFRobot);

         for (Map.Entry<String, URDFLink> stringURDFLinkEntry : allLinksFromURDFRobot.entrySet())
         {
            String linkName = stringURDFLinkEntry.getKey();
            URDFLink link = stringURDFLinkEntry.getValue();

            LinkDescription linkDescription = new LinkDescription(linkName);

            ArrayList<URDFVisual> linkVisuals = new ArrayList<>();
            ArrayList<URDFCollision> linkCollisions = new ArrayList<>();

            URDFInertial inertialInformationForURDFLink = ModelFileLoaderConversionsHelper.getInertialInformationAndPackVisualsAndCollisionsForURDFLink(link, linkVisuals, linkCollisions);
            setupInertialInformationForLink(linkName, linkDescription, inertialInformationForURDFLink);
         }

         for (Map.Entry<String, URDFJoint> stringURDFJointEntry : allJointsFromURDFRobot.entrySet())
         {
            System.out.println(stringURDFJointEntry.getKey());
         }

         for (Map.Entry<String, URDFMaterialGlobal> stringURDFMaterialGlobalEntry : allGlobalMaterialsFromURDFRobot.entrySet())
         {
            System.out.println(stringURDFMaterialGlobalEntry.getKey());
         }

         for (Map.Entry<String, URDFTransmission> stringURDFTransmissionEntry : allTransmissionFromURDFRobot.entrySet())
         {
            System.out.println(stringURDFTransmissionEntry.getKey());
         }
      }
      catch (Throwable e)
      {
         robotDescription = null;
         throw new RuntimeException("Could not load robot model " + modelName, e);
      }

      return robotDescription;
   }

   private void setupInertialInformationForLink(String linkName, LinkDescription linkDescription, URDFInertial inertialInformationForURDFLink)
   {
      if (inertialInformationForURDFLink != null)
      {
         URDFMass mass = inertialInformationForURDFLink.getMass();
         URDFInertia inertia = inertialInformationForURDFLink.getInertia();
         URDFPose origin = inertialInformationForURDFLink.getOrigin();

         parseMassProperties(linkName, linkDescription, mass, origin);

         parseInertiaProperties(linkName, linkDescription, inertia, origin);
      }
      else
      {
         PrintTools.warn(this, "No <intertial> tag for link " + linkName + ", setting all inertial properties to 0.0");
         linkDescription.setMomentOfInertia(0.0, 0.0, 0.0);
         linkDescription.setMass(0.0);
         linkDescription.setCenterOfMassOffset(0.0, 0.0, 0.0);
      }
   }

   private void parseInertiaProperties(String linkName, LinkDescription linkDescription, URDFInertia inertia, URDFPose origin)
   {
      RotationMatrix rotationMatrix = new RotationMatrix();
      Matrix3D momentOfInertia = new Matrix3D();

      if (inertia != null)
      {
         momentOfInertia.set(inertia.getIxx(), inertia.getIxy(), inertia.getIxz(), inertia.getIxy(), inertia.getIyy(), inertia.getIyz(), inertia.getIxz(),
                             inertia.getIyz(), inertia.getIzz());

         if (origin != null)
         {
            String rpyString = origin.getRpy();
            String[] rpySplit = rpyString.split(" ");

            if (rpySplit == null || rpySplit.length != 3)
            {
               PrintTools.warn(this, "Improperly formatted Intertial pose RPY values for link " + linkName + ", skipping transformation");
            }
            else
            {
               try
               {
                  double roll = Double.parseDouble(rpySplit[0]);
                  double pitch = Double.parseDouble(rpySplit[1]);
                  double yaw = Double.parseDouble(rpySplit[2]);

                  rotationMatrix.setYawPitchRoll(yaw, pitch, roll);
               }
               catch (Throwable e)
               {
                  PrintTools.warn(this, "Improperly formatted Intertial pose RPY values for link " + linkName + ", skipping transformation");
               }
            }
         }

         rotationMatrix.transform(momentOfInertia);
         linkDescription.setMomentOfInertia(momentOfInertia);
      }
   }

   private void parseMassProperties(String linkName, LinkDescription linkDescription, URDFMass mass, URDFPose origin)
   {
      if (mass != null)
      {
         linkDescription.setMass(mass.getValue());
      }

      if (origin != null)
      {
         String xyzString = origin.getXyz();
         String[] xyzSplit = xyzString.split(" ");

         if (xyzSplit == null || xyzSplit.length != 3)
         {
            PrintTools.warn(this, "Improperly formatted Intertial pose xyz values for link " + linkName + ", skipping offset");
         }
         else
         {
            try
            {
               {
                  double x = Double.parseDouble(xyzSplit[0]);
                  double y = Double.parseDouble(xyzSplit[1]);
                  double z = Double.parseDouble(xyzSplit[2]);

                  linkDescription.setCenterOfMassOffset(x, y, z);
               }
            }
            catch (Throwable e)
            {
               PrintTools.warn(this, "Improperly formatted Intertial pose xyz values for link " + linkName + ", skipping offset");
            }
         }
      }
   }

   private List<URDFGazebo> getGazeboExtensions(URL modelFileResourceURL) throws XMLStreamException, IOException, JAXBException
   {
      XMLInputFactory xmlInputFactory = XMLInputFactory.newFactory();
      XMLStreamReader xmlStreamReader = xmlInputFactory.createXMLStreamReader(new StreamSource(modelFileResourceURL.openStream()));
      ArrayList<URDFGazebo> gazeboTags = new ArrayList<>();
      Unmarshaller unmarshaller = JAXBContext.newInstance(URDFGazebo.class).createUnmarshaller();

      int eventType = xmlStreamReader.next();
      while (xmlStreamReader.hasNext())
      {
         if (eventType == XMLStreamConstants.START_ELEMENT)
         {
            if (xmlStreamReader.getLocalName().equals("gazebo"))
            {
               JAXBElement<URDFGazebo> unmarshal = unmarshaller.unmarshal(xmlStreamReader, URDFGazebo.class);

               gazeboTags.add(unmarshal.getValue());
            }
            eventType = xmlStreamReader.next();
         }
         else
         {
            eventType = xmlStreamReader.next();
         }
      }
      return gazeboTags;
   }
}
