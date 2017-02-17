package us.ihmc.modelFileLoaders.urdfLoader;

import us.ihmc.modelFileLoaders.ModelFileLoaderConversionsHelper;
import us.ihmc.modelFileLoaders.urdfLoader.xmlDescription.URDFJoint;
import us.ihmc.modelFileLoaders.urdfLoader.xmlDescription.URDFLink;
import us.ihmc.modelFileLoaders.urdfLoader.xmlDescription.URDFRobot;
import us.ihmc.robotics.partNames.JointNameMap;
import us.ihmc.robotics.robotDescription.RobotDescription;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Unmarshaller;
import java.io.InputStream;
import java.util.List;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class RobotDescriptionFromURDFLoader
{
   private final JAXBContext jaxbContext;
   private final Unmarshaller unmarshaller;
   private String modelName;
   private List<String> resourceDirectories;

   public RobotDescriptionFromURDFLoader()
   {
      try
      {
         jaxbContext = JAXBContext.newInstance("us.ihmc.modelFileLoaders.urdfLoader.xmlDescription");
         unmarshaller = jaxbContext.createUnmarshaller();
      }
      catch (JAXBException e)
      {
         throw new RuntimeException(e);
      }
   }

   public RobotDescription loadRobotDescriptionFromURDF(String modelName, InputStream inputStream, List<String> resourceDirectories, JointNameMap jointNameMap, boolean useCollisionMeshes)
   {
      RobotDescription robotDescription;
      try
      {
         robotDescription = new RobotDescription(modelName);
         URDFRobot urdfRobot = (URDFRobot) unmarshaller.unmarshal(inputStream);

         List<URDFJoint> allJointsFromURDFRobot = ModelFileLoaderConversionsHelper.getAllJointsFromURDFRobot(urdfRobot);
         List<URDFLink> allLinksFromURDFRobot = ModelFileLoaderConversionsHelper.getAllLinksFromURDFRobot(urdfRobot);
      }
      catch (Throwable e)
      {
         robotDescription = null;
         throw new RuntimeException("Could not load robot model " + modelName, e);
      }

      return robotDescription;
   }
}
