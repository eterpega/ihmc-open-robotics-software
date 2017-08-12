package us.ihmc.valkyrie.configuration;

public class ValkyrieConfigurationRoot
{
	public static String FOOTPLATE_WITHOUT_TOE_JOINT = "models/val_description/sdf/valkyrie_simfootplate.sdf";
	public static String FOOTPLATE_WITH_TOE_JOINT_HALF = "models/val_description/sdf/valkyrie_simtoejoint_half.sdf";
	public static String FOOTPLATE_WITH_TOE_JOINT_THREEQUATER = "models/val_description/sdf/valkyrie_simtoejoint_threequater.sdf";
	
   public static final String REAL_ROBOT_SDF_FILE = "models/val_description/sdf/valkyrie_sim.sdf";
   
//   public static final String SIM_SDF_FILE = "models/val_description/sdf/valkyrie_sim.sdf";
   public static final String SIM_SDF_FILE = FOOTPLATE_WITH_TOE_JOINT_HALF; // added for toe joint

   public static final boolean VALKYRIE_WITH_ARMS = true;

}
