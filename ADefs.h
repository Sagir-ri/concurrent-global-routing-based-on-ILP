// -*- C++ -*-
//
// $Id: ADefs.h,v 1.4 2008/06/11 09:20:34 jpc Exp $
//
// +-----------------------------------------------------------------+ 
// |        A l l i a n c e   C A D   S y s t e m                    |
// |              S i m p l e   R o u t e r                          |
// |                                                                 |
// |  Author      :                    Jean-Paul CHAPUT              |
// |  E-mail      :       alliance-support@asim.lip6.fr              |
// | =============================================================== |
// |  C++ Header  :       "./ADefs.h"                                |
// +-----------------------------------------------------------------+

#ifndef  __ADefs__
#define  __ADefs__  1

#include  "MDefs.h"
#include  <vector>
#include  <queue>
#include  <set>
#include  <map>
#include  <unordered_map>

// -------------------------------------------------------------------
// Class forward declarations.

class  CAStar;
class  CASimple;
class  CILPRouter;  // New ILP Router class

// -------------------------------------------------------------------
// Module  :  "AAstar.cpp".

// ---------------------------------------------------------------
// AStar trapped exception.
class trapped_astar : public except_done {
public:
    CNet *net;
    
public:
    trapped_astar(CNet *pNet) { net = pNet; }
    
public:
    const char* what() const noexcept {
        return ((char*)"AStar algorithm can't find a path.");
    }
};

// ---------------------------------------------------------------
// Unable to route exception.
class no_route : public except_done {
public:
    CNet *net;
    
public:
    no_route(CNet *pNet) { net = pNet; }
    
public:
    const char* what() const noexcept {
        return ((char*)"No route.");
    }
};

// ---------------------------------------------------------------
// Maximum priority reached exception.
class reach_max_pri : public except_done {
public:
    CNet *net;
    
public:
    reach_max_pri(CNet *pNet) { net = pNet; }
    
public:
    const char* what() const noexcept {
        return ((char*)"Maximum priority reached in AStar.");
    }
};

// ---------------------------------------------------------------
// AStar algorithm class.
class CAStar {
public:
    class CNodeAS;
    class CNodeASSet;
    class CTree;
    class CQueue;

    typedef  list<CNodeAS*>  LNodeAS;

    // Node class for A* algorithm
    class CNodeAS {
    public:
        CDRGrid::iterator  point;
        CNodeAS           *back;
        long               distance;
        long               remains;
        bool               queued;
        bool               tagged;
        bool               intree;
        int                id;

    public:
        bool  operator< (const CNodeAS &other) const;
        CNodeAS(CDRGrid::iterator &pos);

    private:
        static void *operator new(size_t size);
    public:
        static void  operator delete(void *zone);
        static void *operator new(size_t size, CNodeASSet &NS);

    public:
        void  reset();
        void  settree() { intree = true; }
        void  unsettree() { intree = false; }
        void  successors(CNodeASSet &NS, CNet *net, CNodeAS *(*success)[6]);
        bool  isintree() { return (intree); }
    };

    inline static CNodeAS *AS(CDRGrid::iterator  point) {
        return static_cast<CNodeAS*>(point.node().algo);
    }

    // Heap Allocator for CNodeAS
    class CNodeASSet {
    private:
        vector<CNodeAS*> _chunks;
        int              _maxchunk;
        int              _maxalloc;
        int              _maxused;

    public:
        CDRGrid::iterator *target;

    public:
        CNodeASSet();
        ~CNodeASSet();
        void  reset();
        void  check(bool cleared);

        friend class CNodeAS;
    };

    // Routing tree
    class CTree {
    public:
        CNodeASSet *_NS;
        LNodeAS   nodes;
        set<int>  reached;

    public:
        void addnode(CNodeAS *node) {
            node->settree();
            nodes.push_back(node);
        }
        void addterm(CTerm &term);
        void settarget(CDRGrid::iterator &node);
        int  size() { return (reached.size()); }
        void clear();
    };

    // Priority queue
    class CQueue {
        class CQelem {
        public:
            CNodeAS *node;
            CQelem(CNodeAS *pNode=NULL) { node = pNode; }
            bool operator< (const CQelem &other) const {
                return ((*this->node) < (*other.node));
            }
        };

    private:
        priority_queue<CQelem>  queue;

    public:
        CNodeAS* pop();
        void     push(CNodeAS *node);
        bool     empty() { return (queue.empty()); }
        void     reset() { while (!empty()) pop(); }
        void     load(CTree &tree, bool start);
    };

private:
    CNodeASSet _NS;
    CTree      _tree;
    CQueue     _queue;
    bool       _skip;
    bool       _trapped;
    CNodeAS   *_reached;

public:
    CNet     *net;
    long      iterations;
    long      iterations_route;
    long      iterations_reroute;
    CDRGrid  *_drgrid;
    CASimple *_netsched;

public:
    CAStar(CDRGrid *drgrid, CASimple *netsched);

private:
    bool  step();
    bool  nexttarget();
    void  backtrack();
    void  abort();

public:
    void  dump();
    void  clear();
    void  load(CNet *pNet, int delta=0, int expand=0);
    bool  search();
    void  route(CNet *pNet);
    void  check(bool cleared);
};

// -------------------------------------------------------------------
// Module  :  "ASimple.cpp".

class CASimple {
    class CQueue {
        class CQelem {
        public:
            CNet *net;
            CQelem(CNet *pNet=NULL) { net = pNet; }
            bool operator< (const CQelem &other) const {
                return ((*this->net) < (*other.net));
            }
        };

    public:
        priority_queue<CQelem>  queue;

    public:
        CNet* pop();
        void  push(CNet *net) { queue.push(CQelem(net)); }
        bool  empty() { return (queue.empty()); }
        void  reset() { while (!empty()) pop(); }
        void  load(MNet *nets, bool rglobal, bool global);
    };

public:
    CQueue   _queue;
    CAStar   _astar;
    bool     rglobal;
    MNet    *nets;
    long     iterations_route;
    long     iterations_reroute;

public:
    CASimple(MNet *mNets, CDRGrid *drgrid);
    void  queue(CNet *net) { _queue.push(net); }
    bool  step();
    void  global();
    void  local();
    void  stats();
    void  run(bool rtype);
};

// -------------------------------------------------------------------
// Module  :  "AILPRouter.cpp" - New ILP Router with A* integration

class CILPRouter {
public:
    // PathNode structure for enhanced A* pathfinding
    struct PathNode {
        CDRGrid::iterator position;
        double g_cost;      // Cost from start
        double h_cost;      // Heuristic cost to goal
        double f_cost;      // Total cost (g + h)
        int parent_edge;    // Edge direction from parent
        long long parent_key; // Parent node key for reconstruction
        
        PathNode() : g_cost(0), h_cost(0), f_cost(0), parent_edge(-1), parent_key(0) {}
        
        // Comparator for priority queue (min-heap)
        bool operator<(const PathNode& other) const {
            return f_cost > other.f_cost;
        }
        
        // Update total cost
        void updateCost() {
            f_cost = g_cost + h_cost;
        }
    };

    // Net data structure for ILP optimization
    struct NetData {
        CNet* net;
        std::vector<std::vector<int>> candidate_paths;
        std::vector<double> path_costs;
        int selected_path_index;
        
        NetData(CNet* n) : net(n), selected_path_index(-1) {}
    };

    // Edge capacity and cost information
    struct EdgeInfo {
        int capacity;
        int current_usage;
        double base_cost;
        double congestion_cost;
        
        EdgeInfo() : capacity(1), current_usage(0), base_cost(1.0), congestion_cost(0.0) {}
        
        double getTotalCost() const {
            return base_cost + congestion_cost;
        }
        
        bool isAvailable() const {
            return current_usage < capacity;
        }
    };

private:
    // Core data structures
    CDRGrid* _drgrid;
    std::vector<NetData> _net_data;
    std::unordered_map<long long, EdgeInfo> _edge_info;
    
    // Algorithm parameters
    int _max_iterations;
    double _congestion_penalty;
    double _distance_weight;
    int _num_candidate_paths;
    
    // Statistics
    long _total_iterations;
    long _successful_routes;
    long _failed_routes;

public:
    // Constructor
    CILPRouter(CDRGrid* drgrid);
    
    // Destructor
    ~CILPRouter();

private:
    // A* algorithm methods
    int calculateAStarCapacity(CDRGrid::iterator node, int direction);
    double calculateAStarCost(CDRGrid::iterator node, int direction);
    double calculateHeuristic(CDRGrid::iterator current, CDRGrid::iterator goal);
    long long getNodeKey(CDRGrid::iterator node);
    
    void expandNeighborsAStar(
        const PathNode& current,
        CDRGrid::iterator end,
        int strategy,
        std::priority_queue<PathNode>& open_list,
        std::set<long long>& closed_list,
        std::map<long long, PathNode>& node_map
    );
    
    void reconstructPath(
        const PathNode& goal,
        const std::map<long long, PathNode>& node_map,
        std::vector<int>& path
    );
    
    bool isValidAStarPath(
        const std::vector<int>& path,
        CDRGrid::iterator start,
        CDRGrid::iterator end
    );

    // Graph and routing methods
    void buildRoutingGraph();
    void updateEdgeInfo();
    void resetEdgeUsage();
    
    std::vector<int> findPath(
        CDRGrid::iterator start,
        CDRGrid::iterator end,
        int strategy = 0
    );
    
    void generateMultiplePaths(NetData& net_data, int num_paths = 3);
    double calculatePathCost(const std::vector<int>& path);
    
    // ILP optimization methods
    bool solveILP();
    void updateCongestionCosts();
    bool hasConflicts();
    
    // Utility methods
    void clearRouting();
    bool isPathValid(const std::vector<int>& path, CDRGrid::iterator start, CDRGrid::iterator end);
    void applyRouting();

public:
    // Main interface methods
    void loadNets(MNet* nets);
    void addNet(CNet* net);
    bool routeAllNets();
    bool routeNet(CNet* net);
    
    // Configuration methods
    void setMaxIterations(int iterations) { _max_iterations = iterations; }
    void setCongestionPenalty(double penalty) { _congestion_penalty = penalty; }
    void setDistanceWeight(double weight) { _distance_weight = weight; }
    void setNumCandidatePaths(int num_paths) { _num_candidate_paths = num_paths; }
    
    // Statistics and debugging
    void printStatistics();
    void dumpRoutingGraph();
    void checkIntegrity();
    
    // Getters
    long getTotalIterations() const { return _total_iterations; }
    long getSuccessfulRoutes() const { return _successful_routes; }
    long getFailedRoutes() const { return _failed_routes; }
    double getSuccessRate() const {
        long total = _successful_routes + _failed_routes;
        return total > 0 ? (double)_successful_routes / total : 0.0;
    }
};

#endif // __ADefs__