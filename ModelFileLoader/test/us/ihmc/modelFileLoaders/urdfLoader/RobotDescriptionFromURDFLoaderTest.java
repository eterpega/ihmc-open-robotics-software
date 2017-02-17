package us.ihmc.modelFileLoaders.urdfLoader;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.modelFileLoaders.SdfLoader.SDFDescriptionMutator;
import us.ihmc.modelFileLoaders.urdfLoader.xmlDescription.URDFRobot;
import us.ihmc.robotics.partNames.JointNameMap;
import us.ihmc.robotics.robotDescription.RobotDescription;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Unmarshaller;
import java.io.InputStream;
import java.util.List;

import static org.junit.Assert.fail;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class RobotDescriptionFromURDFLoaderTest
{
   private static final String modelName = "atlas";
   private static final List<String> resourceDirectories = (List<String>) null;
   private static final SDFDescriptionMutator mutator = null;
   private static final JointNameMap jointNameMap = null;

   private RobotDescriptionFromURDFLoader loader;
   private InputStream robotModelInputStream;
   private JAXBContext jaxbContext;
   private Unmarshaller unmarshaller;
   private URDFRobot rawURDFRobot;

   @Before
   public void setup() throws JAXBException
   {
      loader = new RobotDescriptionFromURDFLoader();
      robotModelInputStream = getClass().getClassLoader().getResourceAsStream("urdfRobotTest.urdf");

      jaxbContext = JAXBContext.newInstance("us.ihmc.modelFileLoaders.urdfLoader.xmlDescription");
      unmarshaller = jaxbContext.createUnmarshaller();
      rawURDFRobot = (URDFRobot) unmarshaller.unmarshal(getClass().getClassLoader().getResourceAsStream("urdfRobotTest.urdf"));
   }

   @After
   public void teardown()
   {
      loader = null;
      robotModelInputStream = null;
      jaxbContext = null;
      unmarshaller = null;
      rawURDFRobot = null;
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 30000)
   public void testSDFRobotVersusURDFRobot()
   {
      RobotDescription robotDescription = loader.loadRobotDescriptionFromURDF(modelName, robotModelInputStream, resourceDirectories, jointNameMap, true);
      fail();
   }
}