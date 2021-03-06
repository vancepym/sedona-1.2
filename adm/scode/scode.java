//
// Copyright (c) 2007 Tridium, Inc.
// Licensed under the Academic Free License version 3.0
//
// History:
//   15 Feb 07  Brian Frank  Creation
//

package sedonac.scode;

import java.util.*;

/**
 * SCode constants:
 *
 *   NOTE: this file is auto-generated by SCodeGen!!!
 */
public class SCode
{

////////////////////////////////////////////////////////////////
// OpCodes
////////////////////////////////////////////////////////////////

  $opcodes

////////////////////////////////////////////////////////////////
// Misc Constants
////////////////////////////////////////////////////////////////

  $constants

////////////////////////////////////////////////////////////////
// OpCode Argument Types
////////////////////////////////////////////////////////////////

  public static final int noArg     = 0;
  public static final int u1Arg     = 1;
  public static final int u2Arg     = 2;
  public static final int s4Arg     = 3;
  public static final int intArg    = 4;
  public static final int longArg   = 5;
  public static final int floatArg  = 6;
  public static final int doubleArg = 7;
  public static final int strArg    = 8;
  public static final int bufArg    = 9;
  public static final int typeArg   = 10;
  public static final int slotArg   = 11; // slot literal
  public static final int fieldArg  = 12;
  public static final int methodArg = 13;
  public static final int jmpArg    = 14;
  public static final int jmpfarArg = 15;
  public static final int switchArg = 16;
  public static final int arrayArg  = 17; // array literal

////////////////////////////////////////////////////////////////
// SCode Image Flags
////////////////////////////////////////////////////////////////

  public static final int scodeDebug = 0x01;  // is debug compiled in
  public static final int scodeTest  = 0x02;  // are unit tests compiled in
  
////////////////////////////////////////////////////////////////
// Utils
////////////////////////////////////////////////////////////////

  public static int opcode(String name)
  {
    Integer opcode = (Integer)byName.get(name);
    if (opcode == null) return -1;
    return opcode.intValue();
  }

  public static String name(int opcode)
  {
    if (opcode < 0 || opcode >= names.length)
      return opcode + "?";
    else
      return names[opcode];
  }

  private static HashMap byName = new HashMap();
  static
  {
    for (int i=0; i<names.length; ++i)
      byName.put(names[i], new Integer(i));
  }

}
