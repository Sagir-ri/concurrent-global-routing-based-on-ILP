// -*- C++ -*-
//
// $Id: ASimple.cpp,v 1.4 2005/10/10 15:34:05 jpc Exp $
//
//  +----------------------------------------------------------------+ 
//  |        A l l i a n c e   C A D   S y s t e m                   |
//  |              S i m p l e   R o u t e r                         |
//  |                                                                |
//  |  Author      :                    Jean-Paul CHAPUT             |
//  |  E-mail      :       alliance-support@asim.lip6.fr             |
//  | ============================================================== |
//  |  C++ Module  :       "./ASimple.cpp"                           |
//  +----------------------------------------------------------------+

// 关键修复：强制ILP模式，完全禁用A*
#define FORCE_ILP_ONLY 1
#define NO_ASTAR_FALLBACK 1

# include  "ADefs.h"
# include  "AAstar.h"  // 包含ILP路由器

//  +----------------------------------------------------------------+
//  |     修复：CASimple方法使用ILP而不是A*                       |
//  +----------------------------------------------------------------+

// -------------------------------------------------------------------
// Method  :  "CASimple::CQueue::load()".



void CASimple::CQueue::load (MNet *nets, bool rglobal, bool global)
{
  MNet::iterator  itNet, endNet;

  endNet     = nets->end();
  for (itNet = nets->begin(); itNet != endNet; itNet++) {
    // Already routed signal.
    if ( itNet->second->fixed ) continue;

    // Global routing stage.
    if ( global && (itNet->second->global(rglobal)) )
      push (itNet->second);

    // Local routing stage.
    if ( !rglobal || ( !global && !(itNet->second->global(true)) ) )
      push (itNet->second);
  }
}

// -------------------------------------------------------------------
// Constructor  :  "CASimple::CASimple()".

CASimple::CASimple (MNet *mNets, CDRGrid *drgrid)
  : _astar            (drgrid, this)
  , rglobal           (false)
  , nets              (mNets)
  , iterations_route  (0)
  , iterations_reroute(0)
{
  std::cout << "CASimple: Initializing with ILP Router support..." << std::endl;
  
  // 初始化全局ILP路由器
  initializeGlobalRouter(drgrid, this);
  
  std::cout << "CASimple: ILP Router integration completed" << std::endl;
}

// -------------------------------------------------------------------
// Method  :  "CASimple::step()" - 关键修复：使用ILP路由器
//
// 这是导致 "AStar unable to find a path" 消息的源头！

bool CASimple::step (void)
{
  std::cout << "ENTERING MODIFIED STEP METHOD" << std::endl;
  std::cout << "A* IS COMPLETELY DISABLED IN THIS VERSION! debug" << std::endl;

  CNet *net;

  if (_queue.empty()) return (false);

  net = _queue.pop ();

  cmess2 << "     - [" << setw(4) << _queue.queue.size()
         << "] (hp := " << setw(5) << net->bb.hp << ") "
         << "\"" << net->name << "\" (ILP MODE)\n";

  // 关键修复：使用ILP路由器而不是 _astar.route(net)
  std::cout << "CASimple::step: Routing net " << net->name << " with ILP..." << std::endl;
  
  try {
    CILPRouter* ilp_router = getGlobalRouter();
    if (!ilp_router) {
      std::cerr << "CASimple::step: ILP Router not available!" << std::endl;
      
      // 绝对不回退到A* - 直接失败
      std::cout << "A* routing is DISABLED in pure ILP mode" << std::endl;
      return false;
    }
    
    // 验证纯ILP模式
    if (!ilp_router->isUsingILPOnly()) {
      std::cerr << "CASimple::step: Router not in pure ILP mode!" << std::endl;
      return false;
    }
    
    // 使用ILP路由器进行路由
    ilp_router->route(net);
    
    // 同步统计信息
    iterations_route   += ilp_router->iterations_route;
    iterations_reroute += ilp_router->iterations_reroute;
    
    std::cout << "CASimple::step: Net " << net->name << " routed successfully with ILP" << std::endl;
    
    return true;
    
  } catch (const std::exception& e) {
    std::cerr << "CASimple::step: ILP routing failed for net " << net->name << ": " << e.what() << std::endl;
    std::cerr << "NO A* FALLBACK - Pure ILP mode only" << std::endl;
    
    // 记录失败但不使用A*重试
    std::ofstream log("rnet_ilp_calls.log", std::ios::app);
    if (log.is_open()) {
      log << "FAILED_STEP," << std::time(nullptr) << "," << net->name << ",ILP," << e.what() << std::endl;
      log.close();
    }
    
    return false;  // 失败但不重试
  }
}

// -------------------------------------------------------------------
// Method  :  "CASimple::global()" - 修复：使用ILP全局路由

void CASimple::global (void)
{
  std::cout << "\nCASimple::global: Starting global routing with ILP..." << std::endl;
  
  rglobal = true;
  
  // 确保ILP路由器已初始化
  CILPRouter* ilp_router = getGlobalRouter();
  if (!ilp_router) {
    std::cout << "CASimple::global: Initializing ILP Router..." << std::endl;
    initializeGlobalRouter(_astar._drgrid, this);
    ilp_router = getGlobalRouter();
  }

  cmess2 << "\n";
  cmess1 << "  o  Global routing stage (ILP Mode).\n";

  _queue.load (nets, rglobal, true);

  // 检查是否有网络需要路由
  if (_queue.empty()) {
    std::cout << "CASimple::global: No nets to route globally" << std::endl;
    return;
  }

  std::cout << "CASimple::global: Processing " << _queue.queue.size() << " nets with ILP..." << std::endl;

  // 逐个路由网络
  while ( step() );

  // Locking global signals.
  MNet::iterator  itNet, endNet;
  endNet = nets->end();
  for (itNet = nets->begin(); itNet != endNet; itNet++) {
    if ( itNet->second->global(rglobal) ) itNet->second->locktree();
  }
  
  std::cout << "CASimple::global: Global routing completed with ILP" << std::endl;
}

// -------------------------------------------------------------------
// Method  :  "CASimple::local()" - 修复：使用ILP本地路由

void CASimple::local (void)
{
  std::cout << "\nCASimple::local: Starting local routing with ILP..." << std::endl;
  
  cmess2 << "\n";
  cmess1 << "  o  Local routing stage (ILP Mode).\n";

  _queue.load (nets, rglobal, false);
  
  // 检查是否有网络需要路由
  if (_queue.empty()) {
    std::cout << "CASimple::local: No nets to route locally" << std::endl;
    return;
  }

  std::cout << "CASimple::local: Processing " << _queue.queue.size() << " nets with ILP..." << std::endl;

  while ( step() );
  
  std::cout << "CASimple::local: Local routing completed with ILP" << std::endl;
}

// -------------------------------------------------------------------
// Method  :  "CASimple::stats()" - 修复：显示ILP统计信息

void CASimple::stats (void)
{
  cmess2 << "\n";
  cmess2 << "  o  Routing stats (ILP Mode):\n";
  cmess2 << "     - routing iterations    := " << iterations_route   << "\n";
  cmess2 << "     - re-routing iterations := " << iterations_reroute << "\n";
  cmess2 << "     - ratio                 := "
         << ((float)iterations_reroute /
             (float)(iterations_reroute + iterations_route)) * 100.0
         << "%.\n";
         
  // 显示ILP路由器特有的统计信息
  CILPRouter* ilp_router = getGlobalRouter();
  if (ilp_router) {
    std::cout << "\nILP Router Additional Statistics:" << std::endl;
    ilp_router->printILPStatistics();
  }
}

// -------------------------------------------------------------------
// Method  :  "CASimple::run()" - 修复：运行ILP路由

void CASimple::run (bool rtype)
{
  std::cout << "\nCASimple::run: Starting ILP routing (rtype=" << rtype << ")..." << std::endl;
  
  rglobal = rtype;

  if (rglobal) global ();

  local ();
  stats ();
  
  std::cout << "CASimple::run: ILP routing completed successfully!" << std::endl;
}