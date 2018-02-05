package us.ihmc.llama.parameters;

import java.io.IOException;
import java.nio.file.Paths;

import us.ihmc.communication.remote.RemoteParameterSaver;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.llama.LlamaNetClassList;

public class LlamaParameterSaver
{
   public static void main(String[] args) throws IOException, InterruptedException
   {
      RemoteParameterSaver.run(args, Paths.get("IHMCOpenRoboticsSoftware/llama/resources/parameters"), NetworkPorts.CONTROLLER_PORT, new LlamaNetClassList());
   }
}
