//
// Copyright (c) 2007 Tridium, Inc.
// Licensed under the Academic Free License version 3.0
//
// History:
//   16 Aug 07  Brian Frank  Creation
//

package sedona.sox;

import java.io.*;
import java.util.*;
import sedona.*;
import sedona.util.*;
import sedona.sox.SoxClient.TransferListener;

/**
 * FileTransfer manages file get/puts
 */
class FileTransfer
{

//////////////////////////////////////////////////////////////
// Constructor
//////////////////////////////////////////////////////////////

  FileTransfer(SoxClient client, String uri, SoxFile file,
               Properties reqHeaders, TransferListener listener)
  {
    if (reqHeaders == null) reqHeaders = new Properties();

    // default chunk size leaves 17 bytes of header
    // within the ideal packet size:
    //   5 bytes dasp header
    //   3 bytes ack header field
    //   3 bytes ackMore with 1 byte
    //   1 byte sox cmd
    //   1 byte replyNum
    //   2 bytes chunkNum
    //   2 bytes chunkSize
    // Also see SoxCommands.sedona (openFileReq)
    int defaultChunkSize = client.session().idealMax() - 18;

    this.client      = client;
    this.listener    = listener;
    this.uri         = uri;
    this.file        = file;
    this.reqHeaders  = reqHeaders;
    this.offset      = geti(reqHeaders, "offset", 0);
    this.chunkSize   = geti(reqHeaders, "chunkSize", defaultChunkSize);
    this.lock        = new Object();
    this.startTicks  = Env.ticks();
  }

//////////////////////////////////////////////////////////////
// Get
//////////////////////////////////////////////////////////////

  public Properties getFile()
    throws Exception
  {
    file.open("w");
    try
    {
      doGetFile();
      return resHeaders;
    }
    finally
    {
      file.close();
    }
  }

  private void doGetFile()
    throws Exception
  {
    // start a "get" transaction
    this.method = "g";
    this.fileSize = geti(reqHeaders, "fileSize", 0);
    start();

    // wait until we've received the entire file
    lastReceiveTicks = Env.ticks();
    synchronized (lock)
    {
      while (transferedChunks < numChunks)
      {
        // sanity check
        if (client.session()==null)
          throw new IOException("file transfer session disconnected");

        if (Env.ticks() - lastReceiveTicks > client.session().receiveTimeout())
          throw new IOException("file transfer timed out");

        // wait to receive chunks
        try { lock.wait(1000); } catch(Exception e) {}
      }
    }

    // send close message to free transfer
    Msg req = Msg.prepareRequest('z');
    Msg res = client.request(req);
    res.checkResponse('Z');

    // update progress and report done
    progress();
    done();
  }

  void receiveChunk(Msg msg)
  {
    try
    {
      synchronized (lock)
      {
        int cmd           = msg.u1();
        int replyNum      = msg.u1();
        int chunkNum      = msg.u2();
        int thisChunkSize = msg.u2();
        lastReceiveTicks  = Env.ticks();

//        System.out.println("Received chunk# " + chunkNum + " of file");  // DIAG

        // sanity check - should never receive this if not 'k'
        if (cmd != 'k')
        {
          System.out.println("WARNING: This code is hosed up " + (char)cmd);
          return;
        }

        // check that my chunkNum is within expected range
        if (chunkNum >= numChunks)
        {
          System.out.println("WARNING: Received received out of range chunk " +  chunkNum + " >= " + numChunks);
          return;
        }

        // increment our transfer chunk count
        transferedChunks++;

        // write this chunk to the file
        file.write(chunkNum*chunkSize, msg, thisChunkSize);

        // notify the calling thread
        lock.notifyAll();
      }

      // notify application
      progress();
    }
    catch (Exception e)
    {
      e.printStackTrace();
    }
  }

//////////////////////////////////////////////////////////////
// Put
//////////////////////////////////////////////////////////////

  public Properties putFile()
    throws Exception
  {
    file.open("r");
    try
    {
      doPutFile();
      return resHeaders;
    }
    finally
    {
      file.close();
    }
  }

  private void doPutFile()
    throws Exception
  {
    // start a "put" transaction
    this.method   = "p";
    this.fileSize = geti(reqHeaders, "fileSize", file.size());
    start();

    // start sending chunks - the send window will block
    // us automatically to provide flow control
    for (int i=0; i<numChunks || (i==0 && numChunks==0); ++i)
    {
      sendChunk(i);
      transferedChunks++;
      progress();
    }

    // wait for close command
    long waitStart = Env.ticks();
    synchronized (lock)
    {
      while (!closeReceived)
      {
        // sanity check
        if (client.session()==null)
          throw new IOException("file transfer session disconnected");

        if (Env.ticks() - waitStart > client.session().receiveTimeout())
          throw new IOException("file transfer timed out waiting for put close");

        // wait to receive close message
        try { lock.wait(1000); } catch(Exception e) {}
      }
    }

    // update progress and report done
    progress();
    done();
  }

  private void sendChunk(int chunkNum)
    throws Exception
  {
    // compute this specific chunk's size (in case
    // we are processing the last odd-sized chunk)
    int thisChunkSize = chunkSize;
    if (chunkNum == numChunks-1)
    {
      if (fileSize % chunkSize > 0)
        thisChunkSize = fileSize % chunkSize;
      else if (fileSize == 0)
        thisChunkSize = 0;
    }

    Msg req = Msg.prepareRequest('k', 0);
    req.u2(chunkNum);
    req.u2(thisChunkSize);
    file.read(chunkNum*chunkSize, req, thisChunkSize);

    client.send(req);
  }

  void receiveClose(Msg msg)
  {
    synchronized (lock) { closeReceived = true; }
  }

//////////////////////////////////////////////////////////////
// Common
//////////////////////////////////////////////////////////////

  /**
   * Start the file transaction via the 'f' request and response.
   * This is common code for both gets and puts.
   */
  private void start()
    throws Exception
  {
    synchronized (lock)
    {
      // build request
      Msg req = Msg.prepareRequest('f');
      req.str(method);
      req.str(uri);
      req.i4(fileSize);
      req.u2(chunkSize);

      Iterator it = reqHeaders.keySet().iterator();
      while (it.hasNext())
      {
        String key = (String)it.next();
        if (key.equals("fileSize")) continue;

        String val = (String)reqHeaders.get(key);
        req.str(key);
        req.str(val);
      }
      req.u1(0);  // end of headers

      // send request
      Msg res = client.request(req);

      // parse response
      res.checkResponse('F');
      this.fileSize   = res.i4();
      this.chunkSize  = res.u2();
      this.resHeaders = new Properties();
      while (true)
      {
        String key = res.str();
        if (key.equals("")) break;
        String val = res.str();
        resHeaders.put(key, val);
      }
      resHeaders.put("fileSize",   ""+fileSize);
      resHeaders.put("chunkSize",  ""+chunkSize);

      // compute how many chunks we're going to transfer,
      // take into account last remainder chunk
      this.numChunks = fileSize/chunkSize;
      if (fileSize % chunkSize > 0) this.numChunks++;

      // if the file size is zero, then send at least one
      // chunk of zero bytes to keep the server side simple
      if (numChunks == 0 && method.equals("p"))
        numChunks = 1;

      // initialize our transfer chunk count
      transferedChunks = 0;
    }
  }

//////////////////////////////////////////////////////////////
// Utils
//////////////////////////////////////////////////////////////

  /**
   * Get a header property as an int.
   */
  static int geti(Properties headers, String key, int def)
  {
    String val = headers.getProperty(key);
    if (val == null) return def;
    return Integer.parseInt(val);
  }

  /**
   * Get a header property as an boolean.
   */
  static boolean getb(Properties headers, String key, boolean def)
  {
    String val = headers.getProperty(key);
    if (val == null) return def;
    return val.equals("true");
  }

  /**
   * If we have a listener, fire a progress callback.
   */
  private void progress()
  {
    if (listener == null) return;
    try
    {
      listener.progress(transferedChunks*chunkSize, fileSize);
    }
    catch (Throwable e)
    {
      e.printStackTrace();
    }
  }

  /**
   * Called when transfer is complete.
   */
  private void done()
  {
    long dur = Env.ticks() - startTicks;
    if (!client.traceXferStats) return;

    System.out.println();
    System.out.println("Done [" + uri + "]");
    System.out.println("  duration:    " + dur + "ms");
    System.out.println("  fileSize:    " + fileSize + " bytes");
    System.out.println("  chunkSize:   " + chunkSize + " bytes");
    System.out.println("  numChunks:   " + numChunks);
    System.out.println();
  }

//////////////////////////////////////////////////////////////
// Fields
//////////////////////////////////////////////////////////////

  SoxClient client;       // parent client
  TransferListener listener;  // progress callback
  String uri;             // filename to read/write
  SoxFile file;           // local representation of file to read/write
  String method;          // "g" for get and "p" for put
  int fileSize;           // number of bytes in file
  int offset;             // byte offset into file for reading/writing
  int chunkSize;          // number of bytes in chunk (last may be smaller)
  int numChunks;          // number of chunks expected to be transfered
  int transferedChunks;   // number of chunks transfers so far
  Properties reqHeaders;  // client specified request headers
  Properties resHeaders;  // server specified response headers
  long startTicks;        // starting time
  long lastReceiveTicks;  // last chunk received (get only)
  boolean closeReceived;  // have we received closed command (put only)
  Object lock;            // b/w caller and Receiver

}

