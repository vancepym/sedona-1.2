//
// Copyright (c) 2007 Tridium, Inc.
// Licensed under the Academic Free License version 3.0
//
// History:
//   2 Apr 07  Brian Frank  Creation
//

package sedonac.steps;

import java.io.*;
import java.util.*;
import sedona.Env;
import sedona.util.*;
import sedonac.*;
import sedonac.Compiler;
import sedonac.ir.*;
import sedonac.namespace.*;


/**
 * GenNativeTable generates nativetable.c which contains
 * the native lookup tables for a VM stage.
 */
public class GenNativeTable
  extends CompilerStep
{

  public GenNativeTable(Compiler compiler)
  {
    super(compiler);       
    
    for (int i=0; i<compiler.platform.nativePatches.length; ++i)
      patches.put(compiler.platform.nativePatches[i], "");
  }

  public void run()
  {
    File file = new File(compiler.outDir, "nativetable.c");
    log.info("  GenNativeTable [" + file + "]");

    try
    {
      Printer out = new Printer(new PrintWriter(new FileWriter(file)));
      generate(out);
      out.close();
    }
    catch (Exception e)
    {
      throw err("Cannot write stub file", new Location(file), e);
    }
  }

  private void generate(Printer out)
    throws IOException
  {
    findNativeMethods();
    orderKits();
    orderMethods();

    if (compiler.sim)
      findNativeImpls();       // only need to do this if building sim SVM...

    // header
    out.w("//").nl();
    out.w("// Generated by sedonac ").w(Env.version).nl();
    out.w("// ").w(Env.timestamp()).nl();
    out.w("//").nl();
    out.nl();
    out.w("#include \"sedona.h\"").nl();
    out.nl();

    // process each kit
    for (int i=0; i<nativeKits.length; ++i)
    {
      NativeKit kit = nativeKits[i];
      if (kit == null) continue;


      // If building sim SVM, create file containing stubs for all natives 
      //  not supplied in source
      if (compiler.sim)
        genNativeStubFile(kit);


      // header comment
      out.w("////////////////////////////////////////////////////////////////").nl();
      out.w("// ").w(kit.kitName).w(" (kitId=").w(kit.kitId).w(")").nl();
      out.w("////////////////////////////////////////////////////////////////").nl();
      out.nl();

      // forwards
      for (int j=0; j<kit.methods.length; ++j)
        forward(out, kit.methods[j]);


      // table for kit
      out.w("// native table for kit ").w(i).nl();
      out.w("NativeMethod ").w("kitNatives").w(i).w("[] = ").nl();
      out.w("{").nl();
      for (int j=0; j<kit.methods.length; ++j)
      {
        IrMethod m = kit.methods[j];
        out.w("  ").w(TextUtil.pad(toFuncName(m)+",", 30))
            .w("  // ").w(kit.kitId).w("::").w(j).nl();
      }
      out.w("};").nl();
      out.nl();
    }


    // native method table
    out.w("////////////////////////////////////////////////////////////////").nl();
    out.w("// Native Table").nl();
    out.w("////////////////////////////////////////////////////////////////").nl();
    out.nl();
    out.w("NativeMethod* ").w("nativeTable[] = ").nl();
    out.w("{").nl();
    for (int i=0; i<nativeKits.length; ++i)
    {
      NativeKit kit = nativeKits[i];
      String s = kit == null ? "NULL," : "kitNatives"+i+",";
      out.w("  ").w(TextUtil.pad(s, 15)).w("  // " + i).nl();
    }
    out.w("};").nl();
    out.nl();                                  

    // native id check
    out.w("////////////////////////////////////////////////////////////////").nl();
    out.w("// Native Id Check").nl();
    out.w("////////////////////////////////////////////////////////////////").nl();
    out.nl();            
    out.w("#ifdef SCODE_DEBUG").nl();                      
    out.w("int isNativeIdValid(int kitId, int methodId)").nl();
    out.w("{").nl();
    out.w("  switch(kitId)").nl();
    out.w("  {").nl();
    for (int i=0; i<nativeKits.length; ++i)
    {
      NativeKit kit = nativeKits[i];                     
      if (kit == null) continue;
      
      IrMethod[] methods = kit.methods;
      if (methods.length == 0) continue;
      if (methods[methods.length-1] == null)
        throw new IllegalStateException();
      
      out.w("    case ").w(kit.kitId).w(":").nl();
      out.w("      if (methodId >= ").w(methods.length).w(") return 0;").nl();
      out.w("      else return kitNatives").w(kit.kitId).w("[methodId] != NULL;").nl();
    }
    out.w("    default:").nl();
    out.w("       return 0;").nl();
    out.w("  }").nl();
    out.w("}").nl();
    out.w("#endif").nl();                      

    out.nl().nl();
  }


  //
  // Returns true if stub was created for the native that returns platform ID;
  // o/w returns false.
  //
  private boolean stub(Printer out, IrMethod m)
  {
    if (m == null) return;

    //
    // NOTE: If native method returning platform ID has different name, this code 
    //       won't catch it; source code file with actual implementation will need to 
    //       be provided by creator of sim SVM.  stub() returns this boolean so that 
    //       calling function can issue warning if no "doPlatformId" method was created.
    //
    boolean isPlatformIdNative = ( m.name().compareTo("doPlatformId")==0 );

    // Need to include sedonaPlatform.h to get defn of PLATFORM_ID
    if ( isPlatformIdNative )
      out.nl().w("#include \"sedonaPlatform.h\"").nl().nl();

    // Comment with method signature
    out.nl().w("// ").w(m.ret).w(" ").w(m.parent.name).w(".").w(m.name).w("(");
    for (int i=0; i<m.params.length; ++i)
    {
      Type p = m.params[i];
      if (i > 0) out.w(", ");
      out.w(p);
    }
    out.w(")").nl();

    // Stub method always returns 0 for now...  (unless Str)
    //   TODO: consider matching return type to method's actual (m.ret?)
    String ntype = "Cell ";
    String retv  = "  Cell ret;\n  ret.ival = 0;\n  return ret;";

    // If return type is wide, adjust prototype & return stmt
    if ( m.ret.isWide() )
    {
      ntype = "int64_t ";
      retv  = "  return 0LL;";
    }
    else if ( m.ret.qname().compareTo("sys::Str")==0 )
    {
      // If this native returns the platform ID, supply the required code
      if ( isPlatformIdNative )
        retv  = "  Cell ret;\n  ret.aval = PLATFORM_ID;\n  return ret;";

      // All other string natives return empty string
      else   
        retv  = "  Cell ret;\n  ret.aval = \"\";\n  return ret;";
    }

    out.w(ntype).w(toFuncName(m)).w("(SedonaVM* vm, Cell* params)").nl();
    out.w("{").nl();

    // retv is the string that changes w/ret type
    out.w(retv).nl();

    out.w("}").nl().nl().nl();


    // Return true if we just stubbed doPlatformId()
    return isPlatformIdNative;
  }



  private void forward(Printer out, IrMethod m)
  {
    if (m == null) return;

    // Comment with method signature
    out.w("// ").w(m.ret).w(" ")
       .w(m.parent.name).w(".").w(m.name).w("(");
    for (int i=0; i<m.params.length; ++i)
    {
      Type p = m.params[i];
      if (i > 0) out.w(", ");
      out.w(p);
    }
    out.w(")").nl();

    // declaration
    out.w("Cell ").w(toFuncName(m)).w("(SedonaVM* vm, Cell* params);").nl().nl();
  }

  private String toFuncName(IrMethod m)
  {
    if (m == null) return "NULL";
    String s = m.parent.kit.name + "_" + m.parent.name + "_" + m.name;
    if (patches.get(m.qname()) != null) s += "_patch";
    return s;
  }                  
  
////////////////////////////////////////////////////////////////
// Find Native Methods
////////////////////////////////////////////////////////////////

  private void findNativeMethods()
  {
    HashMap map = new HashMap();
    IrKit[] kits = compiler.kits;
    for (int i=0; i<kits.length; ++i)
    {
      IrType[] types = kits[i].types;
      for (int j=0; j<types.length; ++j)
      {
        IrSlot[] slots= types[j].declared;
        for (int k=0; k<slots.length; ++k)
          if (slots[k].isMethod() && slots[k].isNative())
          {
            IrMethod m = (IrMethod)slots[k];
            Integer kitId = new Integer(m.nativeId.kitId);
            NativeKit kit = (NativeKit)map.get(kitId);
            if (kit == null) map.put(kitId, kit = new NativeKit(m));
            kit.acc.add(m);
          }
      }
    }
    this.nativeKits = (NativeKit[])map.values().toArray(new NativeKit[map.size()]);
  }

////////////////////////////////////////////////////////////////
// Order Native Methods
////////////////////////////////////////////////////////////////

  private void orderKits()
  {
    // first figure out the highest kit id;
    int maxKitId = 0;
    for (int i=0; i<nativeKits.length; ++i)
      maxKitId = Math.max(maxKitId, nativeKits[i].kitId);

    // allocate and map ordered array
    NativeKit[] ordered = new NativeKit[maxKitId+1];
    for (int i=0; i<nativeKits.length; ++i)
    {
      NativeKit k = nativeKits[i];
      ordered[k.kitId] = k;
    }

    // save back ordered array
    nativeKits = ordered;
  }

  private void orderMethods()
  {
    for (int i=0; i<nativeKits.length; ++i)
      orderMethods(nativeKits[i]);
  }

  private void orderMethods(NativeKit kit)
  {
    if (kit == null) return;

    // flatten into array
    kit.methods = (IrMethod[])kit.acc.toArray(new IrMethod[kit.acc.size()]);
    kit.acc = null;

    // first figure out the highest method id; and
    // check that all the kitIds are the same
    int kitId = kit.methods[0].nativeId.kitId;
    int maxMethodId = 0;
    for (int i=0; i<kit.methods.length; ++i)
    {
      IrMethod m = kit.methods[i];
      if (m.nativeId.kitId != kitId)
        err("Native kitIds must all be the same", m.nativeId.loc);
      maxMethodId = Math.max(maxMethodId, m.nativeId.methodId);
    }

    // allocate and map ordered array
    IrMethod[] ordered = new IrMethod[maxMethodId+1];
    for (int i=0; i<kit.methods.length; ++i)
    {
      IrMethod m = kit.methods[i];
      ordered[m.nativeId.methodId] = m;
    }

    // save back ordered array
    kit.methods = ordered;
  }


////////////////////////////////////////////////////////////////
// Scan source files for native method implementations
//   (can't use regex/pattern with this Java version)
////////////////////////////////////////////////////////////////

  private void findNativeImpls()
  {
    // Create native method impl list
    suppliedNatives = new ArrayList();

    // Scan all the files in outDir for implemented native methods
    File outDir = compiler.outDir;
    File[] ofiles = outDir.listFiles();

    for (int j=0; j<ofiles.length; j++)
    {
      File ff = ofiles[j];
      String fileContents;

      try
      {
        int blockLevel = 0;

        StreamTokenizer stoker = new StreamTokenizer( new FileReader(ff) );

        stoker.resetSyntax(); 
        stoker.whitespaceChars(0x0, ' '); 
        stoker.wordChars('0', '9'); 
        stoker.whitespaceChars(':', '?'); 
        stoker.wordChars('A', 'Z'); 
        stoker.whitespaceChars('[', '^'); 
        stoker.wordChars('_', '_'); 
        stoker.whitespaceChars('`', '`'); 
        stoker.wordChars('a', 'z'); 
        stoker.ordinaryChar('{'); 
        stoker.wordChars('|', '|'); 
        stoker.ordinaryChar('}'); 
        stoker.wordChars('~', '~'); 
        stoker.quoteChar('\'');
        stoker.quoteChar('\"');
        stoker.slashSlashComments(true);
        stoker.slashStarComments(true);
        stoker.eolIsSignificant(false);

        int nxt = stoker.nextToken();

        while (nxt!=StreamTokenizer.TT_EOF)
        {
          if (nxt!=StreamTokenizer.TT_WORD)
          {
            // Skip code inside {} blocks - single char token
            if (nxt=='{') blockLevel++;  
            if (nxt=='}') blockLevel--; 

            nxt = stoker.nextToken(); 
            continue;
          }

          if (blockLevel>0) 
          {
            nxt = stoker.nextToken(); 
            continue;
          }

          if ( (stoker.sval.indexOf("Cell")>=0) || (stoker.sval.indexOf("int64_t")>=0) )
          {
            nxt = stoker.nextToken(); 

            if (nxt!=StreamTokenizer.TT_WORD) continue;

            // Look for exactly 2 underscores in following token
            int usloc = stoker.sval.indexOf('_');
            if (usloc<0) continue;

            usloc = stoker.sval.indexOf('_', usloc+1);
            if (usloc<0) continue;
              
            usloc = stoker.sval.indexOf('_', usloc+1);
            if (usloc>=0) continue;
              
            suppliedNatives.add(stoker.sval);
          }

          nxt = stoker.nextToken(); 
        }

      }
      catch (FileNotFoundException e)
      {
        String errStr = "File " + ff.getPath() + " not found";
        System.out.println(errStr);
        continue;
      }
      catch (IOException e)
      {
        String errStr = "Exception reading file " + ff.getPath();
        System.out.println(errStr);
        continue;
      }
    }
  }


////////////////////////////////////////////////////////////////
// Create source file with native method stubs
////////////////////////////////////////////////////////////////

  private void genNativeStubFile(NativeKit kit)
    throws IOException
  {
    // Create a new file to put native method stubs into
    //     (do not create file if no stubs to write)
    
    ArrayList stubs = new ArrayList();

    // Create list of methods that need to be stubbed
    for (int j=0; j<kit.methods.length; ++j)
    {
      String mname = toFuncName(kit.methods[j]);
      if ( !suppliedNatives.contains((Object)mname) )
        stubs.add(kit.methods[j]);
    }

    // If no stubs, then don't create file
    if (stubs.size()==0) 
      return;

    // Create the file and write the header
    String nativeFile = kit.kitName + "_native_stubs.c";
    File nfile = new File(compiler.outDir, nativeFile);
    log.info("    GenNativeTable [" + nfile + "]");
    Printer nout = new Printer(new PrintWriter(new FileWriter(nfile)));

    // Native stub file header comment
    nout.w("////////////////////////////////////////////////////////////////").nl();
    nout.w("// ").w(kit.kitName).w(" (kitId=").w(kit.kitId).w(")").nl();
    nout.w("////////////////////////////////////////////////////////////////").nl();
    nout.nl().nl();

    // Header
    nout.w("//").nl();
    nout.w("// Generated by sedonac ").w(Env.version).nl();
    nout.w("// ").w(Env.timestamp()).nl();
    nout.w("//").nl();
    nout.nl();
    nout.w("#include \"sedona.h\"").nl();
    nout.nl();

    boolean foundPlatformIdNative = false;

    // Create stub functions
    for (int j=0; j<stubs.size(); ++j)
      foundPlatformIdNative = stub(nout, stubs[j]);

    // Close the file stream
    nout.close();

    if (!foundPlatformIdNative)
      log.warning(" No stub created that returns PLATFORM_ID."); 
  }



//////////////////////////////////////////////////////////////////////////
// NativeKit
//////////////////////////////////////////////////////////////////////////

  static class NativeKit
  {
    NativeKit(IrMethod m)
    {
      kitName = m.parent.kit.name;
      kitId = m.nativeId.kitId;
    }

    String kitName;
    int kitId;
    ArrayList acc = new ArrayList();
    IrMethod[] methods;
  }

//////////////////////////////////////////////////////////////////////////
// Printer
//////////////////////////////////////////////////////////////////////////

  public static class Printer extends PrintWriter
  {
    public Printer(PrintWriter out) { super(out); }
    public Printer w(Object s) { print(s); return this; }
    public Printer w(int i)    { print(i); return this; }
    public Printer indent()    { print(TextUtil.getSpaces(indent*2)); return this; }
    public Printer nl()        { println(); return this; }
    public int indent;
  }


////////////////////////////////////////////////////////////////
// Fields
////////////////////////////////////////////////////////////////

  NativeKit[] nativeKits;          
  HashMap patches = new HashMap();

  ArrayList suppliedNatives;    // list of native method impls found in source


}
