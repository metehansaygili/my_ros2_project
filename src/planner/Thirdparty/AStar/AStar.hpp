#ifndef ASTAR_HPP
#define ASTAR_HPP

#define MAX_ITER 50000

#include <vector>
#include <cmath>
#include <functional>
#include <algorithm>
#include <iostream>
#include <limits>
#include <queue>
#include <unordered_map>
#include <memory> 
#include <unordered_set>
#include <nanoflann.hpp>

//--------------------------------------------------------------------------------------
// Vec2i: Represents a 2-dimensional vector or coordinate on a grid
//--------------------------------------------------------------------------------------
struct Vec2i {
    int x;  // x-coordinate (column)
    int y;  // y-coordinate (row)

    // Default constructor: initializes coordinates to (0,0)
    Vec2i() : x(0), y(0) {}

    // Parameterized constructor: initializes coordinates to (_x, _y)
    Vec2i(int _x, int _y) : x(_x), y(_y) {}

    // Overload the addition operator to add two vectors coordinate-wise
    Vec2i operator + (const Vec2i& rhs) const {
        return Vec2i{x + rhs.x, y + rhs.y};
    }

    // Equality operator: returns true if both coordinates match exactly
    bool operator == (const Vec2i& other) const {
        return (x == other.x && y == other.y);
    }
    
    // Inequality operator: returns true if either coordinate differs
    bool operator != (const Vec2i& other) const {
        return !(*this == other);
    }
};

//--------------------------------------------------------------------------------------
// Vec2iHash: Hash function for Vec2i to enable its use in unordered_map and unordered_set
//--------------------------------------------------------------------------------------
struct Vec2iHash {
    std::size_t operator()(const Vec2i& v) const {
        // Combines the hash values of x and y coordinates.
        // The bit shifting helps reduce collisions.
        return std::hash<int>()(v.x) ^ (std::hash<int>()(v.y) << 1);
    }
};

struct Pose {
    double x;
    double y;
    double theta; // in radians
    Pose(double _x=0, double _y=0, double _theta=0) : x(_x), y(_y), theta(_theta) {}
};

//Trying to add nanoflann KDTree to centerline points.
struct CenterlinePointCloud {
    std::vector<Vec2i> pts;

    inline size_t kdtree_get_point_count() const {
        return pts.size();
    }

    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        return (dim == 0 ? pts[idx].x : pts[idx].y);
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const { return false; }
};

namespace AStar
{
    using uint = unsigned int; // alias for unsigned int

    //----------------------------------------------------------------------------------
    // Node: Represents a state in the A* search with position and orientation (theta)
    //----------------------------------------------------------------------------------

    struct Node {
        Vec2i coordinates;
        double theta;
        uint G;
        uint H;
        std::shared_ptr<Node> parent;

        Node(const Vec2i& coords, double heading, std::shared_ptr<Node> p = nullptr)
            : coordinates(coords), theta(heading), G(0), H(0), parent(p)
        {}

        inline uint getScore() const {
            return G + H;
        }
    };

    //----------------------------------------------------------------------------------
    // NodeKey: Unique key for a node, combining x, y, and discretized theta
    //----------------------------------------------------------------------------------
    struct NodeKey {
        int x;
        int y;
        int theta; // Discretized angle (in degrees)

        // Equality operator to check if two NodeKey instances represent the same state.
        bool operator==(const NodeKey& other) const {
            return x == other.x && y == other.y && theta == other.theta;
        }
    };

    //----------------------------------------------------------------------------------
    // NodeKeyHash: Hash function for NodeKey, necessary for using it in unordered_map.
    //----------------------------------------------------------------------------------
    struct NodeKeyHash {
        std::size_t operator()(const NodeKey& key) const {
            // Combines the hash values for x, y, and theta.
            std::size_t hx = std::hash<int>()(key.x);
            std::size_t hy = std::hash<int>()(key.y);
            std::size_t ht = std::hash<int>()(key.theta);
            return hx ^ (hy << 1) ^ (ht << 2);
        }
    };

    //----------------------------------------------------------------------------------
    // makeNodeKey: Utility function to generate a NodeKey from a coordinate and theta value.
    //              The theta is rounded and normalized to [0, 360) degrees.
    //----------------------------------------------------------------------------------
    inline NodeKey makeNodeKey(const Vec2i& coord, double theta) {
        NodeKey key;
        key.x = coord.x;
        key.y = coord.y;
        // Round theta to the nearest integer and normalize within the range 0-359
        ////key.theta = static_cast<int>(std::round(theta)) % 360;
        ////if (key.theta < 0)
        ////    key.theta += 360;
        ////return key;
        //This makes it round it to every 2 int so halves the node amount.
        int thetaBin = static_cast<int>(std::round(theta / 2.0)) * 2;
        thetaBin = thetaBin % 360;
        if (thetaBin < 0) thetaBin += 360;
        key.theta = thetaBin;
        return key;
    }

    inline Pose simulate_arc(const Pose& p, double steering_deg, double step, double min_radius)
    {
        Pose out;

        // Convert steering angle to radians
        double steering = steering_deg * M_PI / 180.0;

        // Compute curvature (1 / turning radius)
        double R;
        if (fabs(steering) < 1e-6) {
            R = std::numeric_limits<double>::infinity(); // straight line
        } else {
            R = min_radius / tan(steering); // turning radius
        }

        // Change in heading along the arc
        double dtheta = (fabs(R) < 1e-6) ? 0.0 : step / R;

        if (fabs(steering) < 1e-6) {
            // Straight motion
            out.x = p.x + step * cos(p.theta);
            out.y = p.y + step * sin(p.theta);
            out.theta = p.theta;
        } else {
            // Circular arc motion
            double cx = p.x - R * sin(p.theta); // center x
            double cy = p.y + R * cos(p.theta); // center y

            out.theta = p.theta + dtheta;
            out.x = cx + R * sin(out.theta);
            out.y = cy - R * cos(out.theta);
        }

        // Normalize theta to [0, 2π)
        out.theta = fmod(out.theta, 2.0 * M_PI);
        if (out.theta < 0) out.theta += 2.0 * M_PI;

        return out;
    }

    inline double normalize_angle(double deg) {
        deg = std::fmod(deg, 360.0);
        if (deg < -180.0) deg += 360.0;
        if (deg > 180.0) deg -= 360.0;
        return deg;
    }

    //----------------------------------------------------------------------------------
    // Heuristic functions namespace: Provides different heuristics for A* search.
    //----------------------------------------------------------------------------------
    namespace Heuristic {

        // getDelta: Computes the absolute difference (delta) between two points along each axis.
        inline Vec2i getDelta(const Vec2i& source, const Vec2i& target) {
            return {std::abs(source.x - target.x), std::abs(source.y - target.y)};
        }

        // manhattan: Manhattan distance heuristic (scaled by a factor of 10).
        //            Suitable for grid-based movement.
        inline uint manhattan(const Vec2i& source, const Vec2i& target) {
            auto delta = getDelta(source, target);
            return 10 * (delta.x + delta.y);
        }

        // euclidean: Euclidean distance heuristic (scaled by a factor of 10).
        //            Provides a straight-line distance between two points.
        inline uint euclidean(const Vec2i& source, const Vec2i& target) {
            auto delta = getDelta(source, target);
            double dist = std::sqrt(delta.x * delta.x + delta.y * delta.y);
            return static_cast<uint>(10.0 * dist);
        }

        // nonholonomic: A heuristic for systems with nonholonomic constraints.
        //                Considers both position and orientation (theta) differences.
        inline uint nonholonomic(const Vec2i& sourcePos, double sourceTheta,
                                 const Vec2i& targetPos, double targetTheta)
        {
            // Compute the positional difference using Euclidean distance scaled by 10.
            auto deltaPos = getDelta(sourcePos, targetPos);
            uint costPos = static_cast<uint>(10.0 *
                           std::sqrt(deltaPos.x * deltaPos.x + deltaPos.y * deltaPos.y));

            // Compute the orientation difference (in degrees) and normalize it.
            double diffTheta = std::fabs(sourceTheta - targetTheta);
            if (diffTheta > 180.0) {
                diffTheta = 360.0 - diffTheta;
            }
            // The cost for orientation is taken as the degree difference.
            uint costTheta = static_cast<uint>(diffTheta);

            // Total heuristic cost is the sum of positional and orientation costs.
            return costPos + costTheta;
        }
    }

    //----------------------------------------------------------------------------------
    // NodeComparator: Functor used to compare two nodes based on their total cost (F score).
    //                 This is required for the priority queue in the A* algorithm.
    //----------------------------------------------------------------------------------
    struct NodeComparator {
        bool operator()(const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) {
            // The node with the smaller F score (G+H) should have higher priority.
            return a->getScore() > b->getScore();
        }
    };

    //----------------------------------------------------------------------------------
    // Generator: Main class implementing the A* search algorithm.
    //            It encapsulates world configuration, collision handling, and path finding.
    //----------------------------------------------------------------------------------
    class Generator {
    public:
        // Constructor: Initializes default world size, allowed heading changes,
        // default heuristic function, and clearance (safety margin).
        Generator() {
            worldSize = {27, 27}; // Default grid size
            
            /*IF YOU CHANGE THE DEGREE THING(MAKENODEKEY) to 3 degrees and change heading changes to 18 15 12 9 6 0 ... :
                Car will be able to take longer turns smoother but shorter turns won't be possible.
                If the distance between obstacles > 5-7 meters, than 3 degrees and 18 15 12 9 6 0 setup would work better.
            */
            // Allowed heading changes in degrees for each move.
            headingChanges = {-18, -16, -12, -8, -4, 0, 4, 8, 12, 16, 18};

            min_turning_radius = 5.5;

            // Set default heuristic to nonholonomic which accounts for both position and orientation.
            heuristicFunc = [](const Vec2i& sPos, double sTheta,
                               const Vec2i& tPos, double tTheta)
            {
                return Heuristic::nonholonomic(sPos, sTheta, tPos, tTheta);
            };

            // Clearance: the number of cells around obstacles to avoid.
            clearance = 0;

            // Centerline contributing factor: a factor that increases the radius of curvature
            centerline_contributing_factor = 0.8;
        }

        void setMinTurningRadius(double r) {
                min_turning_radius = r;
            }

        // setWorldSize: Configures the size of the world/grid.
        void setWorldSize(const Vec2i& ws) {
            worldSize = ws;
        }

        // removeCollision: Removes a collision (obstacle) at a specific coordinate.
        void removeCollision(const Vec2i& coord) {
            walls.erase(coord);
        }

        // clearCollisions: Clears all obstacles from the world.
        void clearCollisions() {
            walls.clear();
        }

        // addCollision: Adds an obstacle at a specific coordinate.
        void addCollision(const Vec2i& coord) {
            walls.insert(coord);
        }

        // setClearance: Sets the safety clearance (buffer zone) around obstacles.
        void setClearance(int c) {
            clearance = c;
        }

        // setCurvatureContributingFactor: Sets the factor that increases radius of curvature
        void setCurvatureContributingFactor(double factor) {
            curvature_contributing_factor = factor;
        }

        // Define a function type for 3D heuristic (considering x, y, and theta)
        using Heuristic3D = std::function<uint(const Vec2i&, double,
                                               const Vec2i&, double)>;
        // setHeuristic: Allows setting a custom heuristic function.
        void setHeuristic(Heuristic3D func) {
            heuristicFunc = func;
        }

        //NANOFLANN
        void setCenterline(const std::vector<Vec2i>& centerline_) {
            centerline = centerline_;

            centerlineCloud.pts = centerline_;

            centerlineTree = std::make_unique<KDTree>(
                2, centerlineCloud,
                nanoflann::KDTreeSingleIndexAdaptorParams(10)
            );

            centerlineTree->buildIndex();
        }


        void setCenterlineContributingFactor(double factor){
            this->centerline_contributing_factor = factor;
        }

        //----------------------------------------------------------------------------------
        // findPath: Executes the A* search algorithm to find a path from the source state
        //           to the target state.
        //
        // Parameters:
        // - sourcePos: Starting grid position.
        // - sourceTheta: Starting orientation (in degrees).
        // - targetPos: Goal grid position.
        // - targetTheta: Desired goal orientation (in degrees).
        //
        // Returns:
        // - A vector of Node pointers representing the path from start to goal.
        //   If an exact path is not found, returns the best available path.
        //----------------------------------------------------------------------------------
        std::vector<std::shared_ptr<AStar::Node>> findPath(const Vec2i& sourcePos, double sourceTheta,
                                    const Vec2i& targetPos, double targetTheta)
        {
            std::priority_queue<std::shared_ptr<AStar::Node>, std::vector<std::shared_ptr<AStar::Node>>, NodeComparator> openQueue;
            std::unordered_map<NodeKey, std::shared_ptr<AStar::Node>, NodeKeyHash> openMap;
            std::unordered_set<NodeKey, NodeKeyHash> closedSet;

            std::shared_ptr<AStar::Node> startNode = std::make_shared<Node>(sourcePos, sourceTheta);
            startNode->G = 0;
            startNode->H = heuristicFunc(sourcePos, sourceTheta, targetPos, targetTheta);
            NodeKey startKey = makeNodeKey(sourcePos, sourceTheta);
            openQueue.push(startNode);
            openMap[startKey] = startNode;

            std::shared_ptr<AStar::Node> bestNode = startNode;
            uint bestHeuristic = startNode->H;
            std::shared_ptr<AStar::Node> goalNode = nullptr;

            iter = 0;

            while (!openQueue.empty() && iter < MAX_ITER) {
                iter++;
                std::shared_ptr<AStar::Node> current = openQueue.top();
                openQueue.pop();

                NodeKey currentKey = makeNodeKey(current->coordinates, current->theta);
                if (closedSet.find(currentKey) != closedSet.end())
                    continue;

                closedSet.insert(currentKey);

                if (current->H < bestHeuristic) {
                    bestHeuristic = current->H;
                    bestNode = current;
                }

                double dist_to_goal = std::sqrt(std::pow(current->coordinates.x - targetPos.x, 2) + 
                                                std::pow(current->coordinates.y - targetPos.y, 2));

                double goal_tolerance = 1.0;
                
                // Check orientation difference as well
                double diffTheta = std::fabs(current->theta - targetTheta);
                if (diffTheta > 180.0) {
                    diffTheta = 360.0 - diffTheta;
                }
                double orientation_tolerance = 5.0; // degrees

                if (dist_to_goal <= goal_tolerance && diffTheta <= orientation_tolerance) {
                    goalNode = current;
                    break;
                }

                for (double dTheta : headingChanges){
                    double newTheta = current->theta + dTheta;

                    // --- LIMIT TOTAL TURN ---
                    double deltaFromStart = normalize_angle(newTheta - sourceTheta);
                    if (std::fabs(deltaFromStart) > 180.0) // Adjust as needed, 180 for no limit
                        continue;

                    // --- SIMULATE ARC --- //Minimum turning radius with 18 degrees of steering is 4.63 meters. Also car is around 180 cm so adding 90 cm's
                    Pose currentPose(current->coordinates.x, current->coordinates.y, current->theta * M_PI / 180.0);
                    double step_length = 1 * curvature_contributing_factor;
                    Pose nxt = simulate_arc(currentPose, dTheta, step_length, this->min_turning_radius); // min turning radius from center of gravity

                    // --- ROUND FOR GRID INDEX ---
                    Vec2i newCoord(static_cast<int>(std::round(nxt.x)),
                                static_cast<int>(std::round(nxt.y)));
                    newTheta = nxt.theta * 180.0 / M_PI; // back to degrees

                    if ((newCoord != targetPos) && detectCollision(newCoord))
                        continue;

                    //Added turn cost so tighter turns are more favorable. Not all the turns adds same amount.
                    uint stepCost = 10;
                    uint turnCost = static_cast<uint>(std::fabs(dTheta));
                    if (std::fabs(dTheta) > 1e-9)
                        stepCost += turnCost;
                        //stepCost += 5;

                    uint newG = current->G + stepCost + distance_to_centerline(newCoord); // This is added for the penaltization of the cells that are far from the centerline

                    NodeKey neighborKey = makeNodeKey(newCoord, newTheta);
                    if (closedSet.find(neighborKey) != closedSet.end())
                        continue;


                    auto it = openMap.find(neighborKey);
                    
                    // if (it == openMap.end() || newG < it->second->G) {
                    //     std::shared_ptr<AStar::Node> neighbor = std::make_shared<Node>(newCoord, newTheta, current);
                    //     neighbor->G = newG;
                    //     neighbor->H = heuristicFunc(newCoord, newTheta, targetPos, targetTheta);
                    //     openQueue.push(neighbor);
                    //     openMap[neighborKey] = neighbor;
                    //}

                    //LAZY EXPANSION
                    uint newScore = newG + heuristicFunc(newCoord, newTheta, targetPos, targetTheta);
                    uint oldScore = it != openMap.end() ? it->second->getScore() : std::numeric_limits<uint>::max();

                    // Threshold: only push if improvement is significant
                    // Threshold -> amount of nodes -> algorithm speed // 0 -> %100 -> 1x // 5 -> %90-95 -> 1.05-1.1x // 10 -> %70-80 -> 1.2-1.4x //
                    uint threshold = 5; // you can tune this
                    if (newScore + threshold < oldScore) {
                        std::shared_ptr<Node> neighbor = std::make_shared<Node>(newCoord, newTheta, current);
                        neighbor->G = newG;
                        neighbor->H = heuristicFunc(newCoord, newTheta, targetPos, targetTheta);
                        openQueue.push(neighbor);
                        openMap[neighborKey] = neighbor;
                    }
                }
            }

            if (!goalNode) {
                // std::cout << "Exact goal not reached, using closest node as fallback." << std::endl;
                goalNode = bestNode;
            }

            std::vector<std::shared_ptr<AStar::Node>> path;
            if (goalNode) {
                std::shared_ptr<AStar::Node> current = goalNode;
                while (current) {
                    path.push_back(current);
                    current = current->parent;
                }
                std::reverse(path.begin(), path.end());
            } else {
                std::cout << "Path not found!" << std::endl;
            }

            return path;
        }


    private:

        //NANOFLANN
        using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
            nanoflann::L2_Simple_Adaptor<double, CenterlinePointCloud>,
            CenterlinePointCloud,
            2 // 2D
        >;

        CenterlinePointCloud centerlineCloud;
        std::unique_ptr<KDTree> centerlineTree;

        //----------------------------------------------------------------------------------
        // detectCollision: Checks if a given coordinate is in collision.
        //
        // The function performs two checks:
        // 1) Grid Boundary: Ensures the coordinate is within the world boundaries.
        // 2) Clearance Check: If clearance > 0, checks surrounding cells for obstacles.
        //----------------------------------------------------------------------------------
        bool detectCollision(const Vec2i& coord) const {
            // Check if the coordinate is outside the grid boundaries.
            if (coord.x < 0 || coord.x >= worldSize.x ||
                coord.y < 0 || coord.y >= worldSize.y)
            {
                return true;
            }
            // If clearance is specified, check neighboring cells within the clearance radius.
            for (int dx = -clearance; dx <= clearance; ++dx) {
                for (int dy = -clearance; dy <= clearance; ++dy) {
                    Vec2i checkCell{coord.x + dx, coord.y + dy};
                    // Skip checking if the cell is outside grid boundaries.
                    if (checkCell.x < 0 || checkCell.x >= worldSize.x ||
                        checkCell.y < 0 || checkCell.y >= worldSize.y)
                        return true; // If the cell is out of the bounds that means there can be a collision.
                        //continue;
                    // If an obstacle exists in the surrounding cell, a collision is detected.
                    if (walls.find(checkCell) != walls.end())
                        return true;
                }
            }
            return false; // No collision detected.
        }

        //NANOFLANN
        uint distance_to_centerline(const Vec2i& coord) const {
            if (!centerlineTree || centerlineCloud.pts.empty()) 
                return 0;

            double query[2] = { double(coord.x), double(coord.y) };

            size_t nearest_index;
            double dist_sq;

            nanoflann::KNNResultSet<double> resultSet(1);
            resultSet.init(&nearest_index, &dist_sq);

            nanoflann::SearchParameters params(10);
            centerlineTree->findNeighbors(resultSet, query, params);

            return centerline_contributing_factor * std::sqrt(dist_sq);
        }


    private:
        Vec2i worldSize;  // Dimensions of the grid world.
        // Set of cells that contain obstacles.
        std::unordered_set<Vec2i, Vec2iHash> walls;
        // Allowed heading changes (in degrees) for each movement step.
        std::vector<double> headingChanges;
        // The selected heuristic function used for cost estimation.
        Heuristic3D heuristicFunc;
        // Clearance value: the safety distance (in cells) around obstacles.
        int clearance;
        // Factor that increases radius of curvature
        double curvature_contributing_factor;
        // Iteration counter for debugging and limiting the search.
        uint iter;
        // Centerline point
        std::vector<Vec2i> centerline = {};
        double centerline_contributing_factor;

        double min_turning_radius;
    };

} // end namespace AStar

#endif // ASTAR_HPP