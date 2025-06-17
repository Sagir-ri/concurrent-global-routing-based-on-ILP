// -*- C++ -*-
//
// RNet.cpp - Modified for ILP Router
// 使用与AAstar.h/AAstar.cpp匹配的接口
//

# include  "RDefs.h"

// 强制ILP模式声明
#define FORCE_ILP_ONLY 1
#define NO_ASTAR_ALLOWED 1

// 包含ILP路由器头文件（现在匹配实现）
# include  "AAstar.h"  

//  +----------------------------------------------------------------+
//  |                     ILP路由方法定义                            |
//  +----------------------------------------------------------------+

// -------------------------------------------------------------------
// Modifier  :  "CNet::route()" - ILP实现

void CNet::route (void)
{
  std::cout << "\nRNet::route() - Using ILP Router for net: " << name << std::endl;
  
  // 移除原来的A*代码：
  // int   pri;
  // int   increase, expand;
  // bool  hysteresis, routed;
  // hysteresis = false;
  // increase   = 0;
  // expand     = 0;
  // do {
  //   if (hysteresis) {
  //     pri = 255 + (1 << increase++);
  //     if (increase > 1) expand = 200;
  //   }
  //   else            pri = 0;
  //   ::astar->clear ();
  //   ::astar->load (this, pri, expand);
  //   routed = !::astar->search ();
  //   hysteresis = true;
  // } while ((increase < 15) && !routed);
  // if (routed) ::astar->dump();
  
  // 替换为ILP路由器调用（使用AAstar.cpp中定义的函数）
  CILPRouter* ilp_router = getGlobalRouter();
  
  if (!ilp_router) {
    std::cerr << "RNet::route() - ILP Router not initialized!" << std::endl;
    std::cerr << "Net: " << name << " CANNOT BE ROUTED!" << std::endl;
    throw std::runtime_error("ILP Router required but not available");
  }
  
  // 验证使用的是ILP
  if (!ilp_router->isUsingILPOnly()) {
    std::cerr << "RNet::route() - Router is not in pure ILP mode!" << std::endl;
    throw std::runtime_error("Router must be in pure ILP mode");
  }
  
  std::cout << "RNet::route() - Using ILP Router for net: " << name << std::endl;
  
  try {
    // 直接调用ILP路由器 - 无重试逻辑，无A*回退
    ilp_router->route(this);
    
    std::cout << "RNet::route() - ILP routing completed for net: " << name << std::endl;
    
    // 记录成功日志
    std::ofstream log("rnet_ilp_calls.log", std::ios::app);
    if (log.is_open()) {
      log << "SUCCESS," << std::time(nullptr) << "," << name << ",ILP" << std::endl;
      log.close();
    }
    
  } catch (const std::exception& e) {
    std::cerr << "RNet::route() - ILP routing failed for net: " << name << std::endl;
    std::cerr << "Error: " << e.what() << std::endl;
    std::cerr << "NO A* FALLBACK - ILP ONLY MODE" << std::endl;
    
    // 记录失败日志
    std::ofstream log("rnet_ilp_calls.log", std::ios::app);
    if (log.is_open()) {
      log << "FAILED," << std::time(nullptr) << "," << name << ",ILP," << e.what() << std::endl;
      log.close();
    }
    
    // 在ILP模式下，失败就是失败，不回退到A*
    throw;
  }
}

// -------------------------------------------------------------------
// 添加ILP专用的路由验证方法

void CNet::verifyILPRouting(void)
{
  std::cout << "Verifying ILP routing for net: " << name << std::endl;
  
  CILPRouter* router = getGlobalRouter();
  if (router) {
    auto usage = router->calculateResourceUsage();
    std::cout << "Net " << name << " - Algorithm: " << usage.algorithm_used << std::endl;
    
    if (usage.algorithm_used != "ILP" && usage.algorithm_used != "PURE_ILP") {
      std::cerr << "ERROR: Net " << name << " was not routed with ILP!" << std::endl;
      throw std::runtime_error("Algorithm verification failed");
    }
  }
}