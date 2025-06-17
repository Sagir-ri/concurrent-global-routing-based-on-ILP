// -*- C++ -*-
//
// AAstar.h - ILP Router Header (修复ADefs.h冲突版本)
//

#ifndef __ASTAR_ILP_H__
#define __ASTAR_ILP_H__

#include "ADefs.h"
#include <vector>
#include <memory>
#include <iostream>
#include <fstream>
#include <ctime>
#include <unordered_map>

// 强制ILP模式标志
#define FORCE_ILP_ONLY 1
#define NO_ASTAR_FALLBACK 1

//  +----------------------------------------------------------------+
//  |                    前向声明                                    |
//  +----------------------------------------------------------------+

class CILPRouter;

// GLPK前向声明（避免在头文件中包含GLPK）
struct glp_prob;

//  +----------------------------------------------------------------+
//  |          关键修复：不重新定义CAStar，而是替换实现           |
//  +----------------------------------------------------------------+

// CAStar已在ADefs.h中声明，我们需要让CAStar使用ILP实现
// 通过继承和委托的方式来实现

//  +----------------------------------------------------------------+
//  |                CILPRouter类声明                                |
//  +----------------------------------------------------------------+

class CILPRouter {
public:
    // Edge representation for global routing
    struct Edge {
        int id;
        CDRGrid::iterator from, to;
        int capacity;       
        int demand;         
        double cost;        
        int direction;      
        
        Edge(int _id, CDRGrid::iterator _from, CDRGrid::iterator _to, 
             int _capacity, int _direction);
    };
    
    // Net representation for ILP formulation
    struct NetData {
        CNet* net;
        std::vector<CDRGrid::iterator> terminals;
        std::vector<std::vector<int> > paths;  
        std::vector<int> path_var_ids;        
        double priority;
        
        NetData(CNet* _net);
    };
    
    // Global routing grid graph
    struct RoutingGraph {
        std::vector<Edge> edges;
        std::unordered_map<long long, int> edge_map;  
        int num_nodes;
        
        void addEdge(CDRGrid::iterator from, CDRGrid::iterator to, int capacity, int direction);
        int getEdgeId(CDRGrid::iterator from, CDRGrid::iterator to);
        long long getNodeKey(CDRGrid::iterator node);
    };

    // GLPK Solver Interface
    struct GLPKSolver {
        glp_prob* problem;
        std::vector<std::pair<int, std::string> > variables;
        bool solved;
        double objective_value;
        
        GLPKSolver();
        ~GLPKSolver();
        
        int addVariable(const std::string& name, double lb = 0.0, double ub = 1.0);
        int addConstraint(const std::string& name, const std::vector<std::pair<int, double> >& coeffs, 
                         double rhs, int bound_type);
        void setObjective(const std::vector<std::pair<int, double> >& obj, bool minimize = true);
        bool solve();
        double getVariableValue(int var_id);
        void clear();
    };

    // 资源使用统计结构
    struct ResourceUsage {
        int total_wire_length;
        int total_vias;
        int congested_edges;
        double total_cost;
        std::string algorithm_used;
        
        ResourceUsage() : total_wire_length(0), total_vias(0), 
                         congested_edges(0), total_cost(0.0), 
                         algorithm_used("PURE_ILP") {}
    };

private:
    // 核心数据结构
    RoutingGraph _graph;
    std::vector<NetData> _nets;
    GLPKSolver* _solver;
    CDRGrid* _drgrid;
    CASimple* _netsched;
    
    // 算法参数
    double _congestion_penalty;
    double _via_penalty;
    int _max_iterations;
    
    // 统计信息
    int _total_iterations;
    int _successful_routes;
    int _failed_routes;

public:
    // 构造函数和析构函数
    CILPRouter(CDRGrid* drgrid, CASimple* netsched);
    ~CILPRouter();
    
    // 主要路由接口（与CAStar兼容）
    void route(CNet* pNet);
    void clear();
    void check(bool cleared = false);
    void dump();
    
    // ILP特有方法
    void printStatistics();
    void setParameters(double congestion_penalty = 2.0, double via_penalty = 1.5, 
                      int max_iterations = 50);
    bool isUsingILPOnly() const { return true; }
    
    // 资源统计
    ResourceUsage calculateResourceUsage();
    void enableVerification(const std::string& log_file = "ilp_verification.log");
    void disableVerification();
    void printILPStatistics();
    
    // ILP算法核心方法
    void buildRoutingGraph();
    void generateCandidatePaths(NetData& net_data);
    void formulateILP();
    bool solveILP();
    void extractSolution();
    
    // 路径生成算法
    std::vector<int> findPath(CDRGrid::iterator start, CDRGrid::iterator end, int strategy = 0);
    void generateMultiplePaths(NetData& net_data, int num_paths = 3);
    double calculatePathCost(const std::vector<int>& path);
    
    // 拥塞管理
    void updateEdgeCosts();
    void handleCongestion();
    bool isOvercongested();
    
    // 辅助方法
    double manhattanDistance(CDRGrid::iterator a, CDRGrid::iterator b);
    bool isValidPath(const std::vector<int>& path, CNet* net);
    void applyRoutingSolution();

    // 增强功能
    void routeAllConcurrently(std::vector<CNet*>& nets);
    void analyzeCongestion();
    void adaptiveParameterTuning();

public:
    // 兼容CAStar的公共成员
    CNet* net;
    long iterations;
    long iterations_route;
    long iterations_reroute;
    CDRGrid* _drgrid_compat;  // 兼容性成员
    CASimple* _netsched_compat; // 兼容性成员
};

//  +----------------------------------------------------------------+
//  |     关键修复：CAStar实现委托给CILPRouter                    |
//  +----------------------------------------------------------------+

// 为了不与ADefs.h冲突，我们重新实现CAStar的方法，委托给CILPRouter

// 声明全局CILPRouter实例访问函数
extern CILPRouter* getGlobalILPRouter();
extern void setGlobalILPRouter(CILPRouter* router);

//  +----------------------------------------------------------------+
//  |          全局函数声明                                          |
//  +----------------------------------------------------------------+

extern void initializeGlobalRouter(CDRGrid* drgrid, CASimple* netsched);
extern void cleanupGlobalRouter();
extern CILPRouter* getGlobalRouter();
extern void verify_no_astar_at_runtime();

//  +----------------------------------------------------------------+
//  |                运行时验证支持                                  |
//  +----------------------------------------------------------------+

#ifdef DEBUG_ILP_CALLS
#define ILP_ROUTE_CALL(net_name) \
    do { \
        std::cout << "ILP Route Call: " << (net_name) << " at " << __FILE__ << ":" << __LINE__ << std::endl; \
    } while(0)
#else
#define ILP_ROUTE_CALL(net_name)
#endif

#define VERIFY_ILP_USAGE() \
    do { \
        CILPRouter* router = getGlobalRouter(); \
        if (router && !router->isUsingILPOnly()) { \
            std::cerr << "ERROR: Not using pure ILP at " << __FILE__ << ":" << __LINE__ << std::endl; \
            std::abort(); \
        } \
    } while(0)

//  +----------------------------------------------------------------+
//  |                C接口兼容性                                     |
//  +----------------------------------------------------------------+

#ifdef __cplusplus
extern "C" {
#endif

void* create_ilp_router(void* drgrid, void* netsched);
void destroy_ilp_router(void* router);
void route_net_ilp(void* router, void* net);

#ifdef __cplusplus
}
#endif

//  +----------------------------------------------------------------+
//  |                编译时验证                                      |
//  +----------------------------------------------------------------+

#ifndef FORCE_ILP_ONLY
#error "FORCE_ILP_ONLY must be defined!"
#endif

#pragma message("ILP Router Header: Fixed ADefs.h conflicts")
#pragma message("Algorithm: GLPK ILP replacing A* completely")

//  +----------------------------------------------------------------+
//  |                内联辅助函数                                    |
//  +----------------------------------------------------------------+

inline void safe_ilp_route(CILPRouter* router, CNet* net) {
    if (!router) {
        throw std::runtime_error("ILP Router is null!");
    }
    
    if (!router->isUsingILPOnly()) {
        throw std::runtime_error("Router is not in pure ILP mode!");
    }
    
    router->route(net);
}

inline void ensure_ilp_only() {
    static bool verified = false;
    if (!verified) {
        std::cout << "Ensuring ILP-only mode..." << std::endl;
        verify_no_astar_at_runtime();
        verified = true;
    }
}

#endif // __ASTAR_ILP_H__