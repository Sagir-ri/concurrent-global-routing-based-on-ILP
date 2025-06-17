// -*- C++ -*-
//
// nero.cpp - Modified for ILP Router
// 主程序入口修改，使用匹配的接口
//

#define FORCE_ILP_ONLY 1
#define NO_ASTAR_FALLBACK 1

# include  "RDefs.h"
# include  "AAstar.h"  // 包含修正的头文件

# ifdef __CYGWIN__
extern "C" {
  int getopt ( int argc, char * const argv[], const char *optstring );
}
# endif

// -------------------------------------------------------------------
// Local Namespace.

namespace {

  // ---------------------------------------------------------------
  // Local variables.

  static CRBox *crbox = NULL;

  // ---------------------------------------------------------------
  // Local functions.

  static void help   (void);
  static void serial (void);

//  +----------------------------------------------------------------+
//  |                    Functions Definitions                       |
//  +----------------------------------------------------------------+

// -------------------------------------------------------------------
// Function  :  "help()".

static void help (void)
{
  cout << "\n"
       << "  NeRo with ILP Router (A* Replaced)\n"
       << "  o  Usage : \"nero [-h] [-v] [-V] [-c] [-2] [-3] [-4] [-5] [-6]\n"
       << "                   [-H] [-L] [-G] [-p <placement>] <netlist> <layout>"
       <<                    "\"\n\n"
       << "             \"nero [--help] [--verbose] [--very-verbose]\n"
       << "                   [--core-dump] [--half-pitch] [--rotate] [--global]"
       << "                   [--local] [--place <placement>] <netlist> <layout>\n"
       << "                  \"\n\n"
       << "  WARNING: This version uses ILP routing instead of A*!\n"
       << "  Algorithm: GLPK Integer Linear Programming\n\n"
       << "  o  Options :\n"
       << "     [-h|--help]         := Print this message.\n"
       << "     [-v|--verbose]      := Be verbose.\n"
       << "     [-V|--very-verbose] := Be much more verbose...\n"
       << "     [-c|--core-dump]    := Generate core dump if an internal "
       <<         "error occurs.\n"
       << "     [-H|--half-pitch]   := First track is at half pitch (both X & Y).\n"
       << "     [-x|--half-pitch-x] := First track is at half pitch (X).\n"
       << "     [-y|--half-pitch-y] := First track is at half pitch (Y).\n"
       << "     [-R|--rotate]       := Exchange preferred routing directions.\n"
       << "     [-2|--layers-2]     := Use only 2 routing layers.\n"
       << "     [-3|--layers-3]     := Use only 3 routing layers.\n"
       << "     [-4|--layers-4]     := Use only 4 routing layers.\n"
       << "     [-5|--layers-5]     := Use only 5 routing layers.\n"
       << "     [-6|--layers-6]     := Use only 6 routing layers.\n"
       << "     [-G|--global]       := Force use of global routing (ILP).\n"
       << "     [-L|--local]        := Use local routing (still ILP-based).\n"
       << "     [-N]--netset <netset>]   :=\n"
       << "         Route only this subset of nets.\n"
       << "     [-p|--place <placement>] :=\n"
       << "         Name of the layout file holding the placement, without\n"
       << "         extention. If ommited the layout file is assumed to have\n"
       << "         the same name as the netlist file <netlist>.\n"
       << "     <netlist>           := Name of the netlist file (mandatory).\n"
       << "     <layout>            := Name of the output layout file (mandatory).\n"
       << "\n"
       << "  ILP Router Features:\n"
       << "     - Integer Linear Programming optimization\n"
       << "     - Concurrent multi-net routing\n"
       << "     - Advanced congestion management\n"
       << "     - GLPK-based mathematical optimization\n"
       << "\n";
}

// -------------------------------------------------------------------
// Function  :  "serial()".

static void serial (void)
{
  cout << "                                S/N 20250616.ILP\n";
  cout << "                    ILP ROUTING EDITION\n";
}

  // ---------------------------------------------------------------
  // End of Local Namespace.

}

// -------------------------------------------------------------------
// Function  :  "emergency()".

void emergency (void)
{
  string  name = "emergency";

  std::cout << "\nNERO: Emergency backup triggered..." << std::endl;

  try {
    if (crbox) {
      crbox->mbksave (name);
      std::cout << "Emergency backup saved as: " << name << std::endl;
    }
    
    // 保存ILP统计到紧急文件
    CILPRouter* router = getGlobalRouter();
    if (router) {
      std::ofstream emergency_log("emergency_ilp_stats.log");
      if (emergency_log.is_open()) {
        emergency_log << "Emergency backup triggered\n";
        emergency_log << "Timestamp: " << std::time(nullptr) << "\n";
        emergency_log << "Router type: ILP\n";
        
        auto usage = router->calculateResourceUsage();
        emergency_log << "Algorithm used: " << usage.algorithm_used << "\n";
        emergency_log << "Wire length: " << usage.total_wire_length << "\n";
        emergency_log << "Vias: " << usage.total_vias << "\n";
        emergency_log << "Total cost: " << usage.total_cost << "\n";
        
        emergency_log.close();
        std::cout << "ILP statistics saved to emergency_ilp_stats.log" << std::endl;
      }
    }
  }
  catch (...) {
    cerr << herr ("\n");
    cerr << "  An exception have occurs in the emergency backup function itself !\n";
    cerr << "  Sorry, can't save the partially routed figure.\n\n";
    cerr << "  This is a bug. Please e-mail to \"alliance-users@asim.lip6.fr\".\n\n";
  }
}

// -------------------------------------------------------------------
// Function  :  "main()".

int  main (int argc, char *argv[])
{
  COpts        options;
  MBK::CFig   *fig;
  string       name_lofig, name_placed, name_routed;
  int          layers, global;
  set<string>* netSet = NULL;

  try {
    options.add ("v", "verbose");
    options.add ("V", "very-verbose");
    options.add ("h", "help");
    options.add ("c", "coredump");
    options.add ("H", "half-pitch");
    options.add ("x", "half-pitch-x");
    options.add ("y", "half-pitch-y");
    options.add ("R", "rotate");
    options.add ("2", "layers-2");
    options.add ("3", "layers-3");
    options.add ("4", "layers-4");
    options.add ("5", "layers-5");
    options.add ("6", "layers-6");
    options.add ("G", "global");
    options.add ("L", "local");
    options.add ("p", "place", true);
    options.add ("N", "netset", true);
    options.getopts (argc, argv);

    if (options["c"]->parsed) interrupt.coredump = true;

    cmess0.setVL (0);
    if (options["v"]->parsed) cmess0.setVL (1);
    if (options["V"]->parsed) cmess0.setVL (2);

    // 注释掉并替换为：
/*
MBK::alliancebanner ( "NeRo-ILP"
                    , VERSION
                    , "Negotiating Router with ILP"
                    , "2025"
                    , ALLIANCE_VERSION
                    );
*/

// 替换为简单的输出
    std::cout << "\n  NeRo-ILP " << VERSION << " - Negotiating Router with ILP" << std::endl;
    std::cout << "  Copyright (c) 2025, ILP Router Edition" << std::endl;
    
    if ((cmess0.getVL () > 0) || options["h"]->parsed ) {
      serial ();
      cmess1 << "\n";
    }

    if (options["h"]->parsed) {
      help ();
      exit (0);
    }

    if (options.tVals.size() < 2) {
      cerr << herr ("nero:\n");
      cerr << "  Missing mandatory argument <netlist> or <routed> (or both)\n";
      cerr << "\n";
      help ();
      throw except_done ();
    }

    name_lofig  = options.tVals[0];
    name_routed = options.tVals[1];

    if (options["p"]->parsed) {
      name_placed = options["p"]->value;
    } else {
      name_placed = name_lofig;
    }

    if (options["N"]->parsed) {
      string fileNetSet = options["N"]->value;
      cout << "File: " << fileNetSet << endl;
      FILE* file = fopen ( fileNetSet.c_str(), "r" );
      if ( file ) {
        cout << "File Successfully opened." << endl;
        netSet = new set<string>();
        char buffer[2048];
        while ( !feof(file) ) {
          fgets ( buffer, 2048, file );
          size_t length = strlen(buffer);
          if ( buffer[length-1] == '\n' )
            buffer[length-1] = '\0';
          netSet->insert ( buffer );
        }
        fclose ( file );
      }
    }

    layers = 3;
    if (options["2"]->parsed) layers = 3;
    if (options["3"]->parsed) layers = 4;
    if (options["4"]->parsed) layers = 5;
    if (options["5"]->parsed) layers = 6;
    if (options["6"]->parsed) layers = 7;

    global = D::ROUTING_CHOOSE;
    if (options["L"]->parsed) { global = D::ROUTING_LOCAL; }
    if (options["G"]->parsed) {
      global = D::ROUTING_GLOBAL;
      if (layers < 5) layers = 5;
    }

    cmess1 << MBK::env;

    // 添加ILP路由器初始化提示
    std::cout << "\nNERO: Starting with ILP Router..." << std::endl;
    std::cout << "Note: A* algorithm has been replaced with ILP" << std::endl;

    fig   = new MBK::CFig (name_lofig, name_placed);
    crbox = new CRBox ();
    
    std::cout << "NERO: Loading design..." << std::endl;
    
    crbox->mbkload ( fig
                   , layers
                   , 4
                   , global
                   , options["H"]->parsed
                   , options["x"]->parsed
                   , options["y"]->parsed
                   , options["R"]->parsed
                   , netSet );
    
    std::cout << "NERO: Starting routing with ILP..." << std::endl;
    crbox->route ();  // 这将调用我们修改的RNet::route()
    
    std::cout << "NERO: Saving routed design..." << std::endl;
    crbox->mbksave (name_routed);
    
    // 最终统计
    std::cout << "\nNERO: Final Statistics..." << std::endl;
    CILPRouter* router = getGlobalRouter();
    if (router) {
      router->printILPStatistics();
    }

    if ( netSet ) delete netSet;
    
    // 清理
    cleanupGlobalRouter();
  }
  catch (e_zupper &e) {
    cerr << "\n\n"
         << "  First \"double pitch\" layer must be at least equal to ALU5 "
         << "(here : " << MBK::env.z2alu (e.zupper) << ").\n\n"
         << endl;
    cleanupGlobalRouter();
    exit (1);
  }
  catch (bad_grab &e) {
    cerr << herr ("\n");
    cerr << "  Net \"" << e.netName << "\" attempt to grab node ("
         << e.x << "," << e.y << "," << e.z << "),\n"
         << "  which belongs to net \"" << e.ownerName << "\".\n" << endl;
    cerr << "  (nodepri := " << e.nodepri
         << "  , pri := " << e.pri << ", terminal := " << e.terminal
         << ", ident := " << e.ident << ")\n" << endl;
    emergency ();
    cleanupGlobalRouter();
    exit (1);
  }
  catch (dup_term &e) {
    cerr << "  Duplicated terminal node for " << e.name
         << " at " << e.node << endl;
    cleanupGlobalRouter();
    exit (1);
  }
  catch (reach_max_pri &e) {
    cerr << "\n\n"
         << "  ILP Router failed, NeRo was unable to route this"
         << "  design.\n  ILP optimization failed for net " 
         << "\"" << e.net->name << "\".\n\n";
    cerr << "  Note: A* fallback is not available in ILP mode\n\n";
    emergency ();
    cleanupGlobalRouter();
    exit (1);
  }
  catch (except_done &e) {
    cerr << e.what () << endl;
    emergency ();
    cleanupGlobalRouter();
    exit (1);
  }
  catch (...) {
    cerr << herr ("\n");
    cerr << "  An unexpected exception have occurs in ILP routing.\n\n";
    cerr << "  This is a bug. Please e-mail to \"alliance-users@asim.lip6.fr\".\n\n";
    cleanupGlobalRouter();
    exit (1);
  }

  // 成功完成
  cmess1 << "\n\n";
  std::cout << "NERO: ILP routing completed successfully!" << std::endl;
  std::cout << "Check rnet_ilp_calls.log for detailed routing information" << std::endl;
                                                              
  exit (0);
}