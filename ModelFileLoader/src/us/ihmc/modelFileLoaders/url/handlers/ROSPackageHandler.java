package us.ihmc.modelFileLoaders.url.handlers;

import java.io.IOException;
import java.io.InputStream;
import java.net.URL;
import java.net.URLConnection;
import java.net.URLStreamHandler;

/**
 * Even though {@link ROSPackageHandler} and {@link GazeboModelHandler} do the exact
 * same thing, we have to make them separate classes because of the way Java URL
 * handlers work under the hood. Lame.
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class ROSPackageHandler extends URLStreamHandler
{
   @Override
   protected URLConnection openConnection(URL u) throws IOException
   {
      return new URLConnection(u)
      {
         @Override
         public void connect() throws IOException
         {

         }

         @Override
         public InputStream getInputStream() throws IOException
         {
            return getClass().getClassLoader().getResourceAsStream(u.getAuthority() + u.getPath());
         }
      };
   }
}
