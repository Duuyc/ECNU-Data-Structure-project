//
// Created by 刘凯源 on 24-10-13.
//
#include "Algorithm.h"
#include "GraphException.h"

namespace Graph {
    namespace Algorithm {
        std::list<Vertex> GetCircuit(LGraph& graph, Vertex start) {
            if (!graph.exist_vertex(start))return {};//出错？
            
            int n = graph.List().size();
            std::vector<HeadNode> ver_list = graph.List();
            std::vector<int> visited(n, 0);
            std::vector<Vertex> parent(n, 0);
            std::vector<Vertex> idx(n, 0);
            std::list<Vertex> circuit{};

            std::stack<Vertex> s;

            visited[start] = 1;
            s.push(start);

            while (!s.empty()) {
                Vertex u = s.top();

                if (idx[u] < ver_list[u].adj.size()) {
                    auto it = ver_list[u].adj.begin();
                    std::advance(it, idx[u]++);
                    Vertex v = (*it).dest;
                    if (visited[v] == 0) {
                        parent[v] = u;
                        visited[v] = 1;
                        s.push(v);
                    }
                    else if (v==start&&s.size()>=2) {
                        circuit.push_back(start);
                        Vertex x = u;
                        while (x != start) {
                            circuit.push_front(x);
                            x = parent[x];
                        }
                        circuit.push_front(x);
                        return circuit;
                    }
                }
                else {
                    visited[u] = 2;
                    s.pop();
                }
            }
            return circuit;
        }//从给定点出发获得一条回路

        std::list<Vertex> EulerCircle(const LGraph& graph) {
            if (!HaveEulerCircle(graph))return {};//出错？

            LGraph temp_graph = graph;
            int n = temp_graph.List().size();
            std::vector<HeadNode> ver_list = temp_graph.List();
            std::list<Vertex> circuit;
            std::stack<Vertex> s;

            for (int i = 0; i < n; i++) {
                s.push(i);
                break;
            }

            while (!s.empty()) {
                Vertex u = s.top();
                if (!ver_list.empty()) {
                    Vertex v = ver_list[u].adj.begin()->dest;
                    if (temp_graph.exist_vertex(v)) {
                        temp_graph.DeleteEdge(u, v);
                        s.push(v);
                    }
                }
                else {
                    circuit.push_back(u);
                    s.pop();
                }
            }

            std::reverse(circuit.begin(), circuit.end());

            return circuit;
        }//获取欧拉回路,你可以使用GetCircuit函数

        bool HaveEulerCircle(const LGraph& graph) {
            if (!IsConnected(graph))return false;

            int count = 0;
            int n = graph.List().size();
            for (int i = 0; i < n; i++) {
                if (graph.List()[i].adj.size() % 2 != 0)count++;
            }

            return count==0||count==2;
        }//判断是否有欧拉回路

        void BFSv(const LGraph& graph, Vertex v, std::vector<bool>& visited) {
            if (!graph.exist_vertex(v))return;//出错?

            /*int n = graph.List().size();
            visited.assign(n, false);*/
            //visited事先处理好
            visited[v] = true;

            std::queue<Vertex> q;
            q.push(v);
            while (!q.empty()) {
                Vertex u = q.front();
                q.pop();
                //访问
                for (const EdgeNode& it : graph.List()[u].adj) {
                    if (!visited[it.dest]) {
                        q.push(it.dest);
                        visited[it.dest] = true;
                    }
                }
            }
        }//广度优先搜索整个图

        bool IsConnected(const LGraph& graph) {
            int n = graph.List().size();
            int component = 0;
            std::vector<bool> visited(n,false);
            for (int i = 0; i < n; i++) {
                if (!visited[i]) {
                    BFSv(graph, i, visited);
                    component++;
                }
            }

            return component==1;
        }//判断图是否联通

        int GetShortestPath(const LGraph& graph, std::vector<int>& pre, const std::string& vertex_name_x, const std::string& vertex_name_y) {
            if (!graph.exist_vertex(vertex_name_x) || !graph.exist_vertex(vertex_name_y))throw GraphException("节点不存在！");
            
            int n = graph.VertexCount();
            std::vector<HeadNode> ver_list = graph.List();
            pre.assign(n, INT_MAX);//
            int x_id = graph.Map().find(vertex_name_x)->second;
            int y_id = graph.Map().find(vertex_name_y)->second;

            std::vector<long long> dist(n, INT_MAX);
            pre[x_id] = x_id;
            dist[x_id] = 0;

            std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> que;
            que.push({ dist[x_id],x_id });

            while (!que.empty()) {
                auto [current_dist, u] = que.top();
                que.pop();

                if (current_dist > dist[u])continue;
                for (EdgeNode& it : ver_list[u].adj) {
                    Vertex v = it.dest;
                    GElemSet w = it.weight;
                    if (dist[u] + w < dist[v]) {
                        dist[v] = dist[u] + w;
                        pre[v] = u;
                        que.push({ dist[v],v });
                    }
                }
            }

            if (dist[y_id] == INT_MAX)throw GraphException("不存在一条通路");
            else return dist[y_id];
        }//获取两点之间的最短路径

        int TopologicalShortestPath(const LGraph& graph, std::vector<std::string> path) {
            //有向无环图？？
            int n = path.size();
            std::vector<HeadNode> ver_list = graph.List();
            std::vector<int> dist(n, INT_MAX);

            dist[0] = 0;

            for (int i = 0; i < n; i++) {
                std::string name = path[i];
                Vertex u = graph.Map().find(name)->second;
                for (EdgeNode& e : ver_list[u].adj) {
                    GElemSet w = e.weight;
                    Vertex v = e.dest;
                    if (dist[u] + w < dist[v]) {
                        dist[v] = dist[u] + w;
                    }
                }
            }

            return dist.back();
        }//获取拓扑受限的最短路径，拓扑序由path给出

        struct cmp_EdgeNode {
            bool operator()(const EdgeNode& a, const EdgeNode& b) {
                return a.weight > b.weight;
            }
        };

        std::vector<EdgeNode> MinimumSpanningTree(const LGraph& graph) {
            int n = graph.VertexCount();
            std::vector<HeadNode> ver_list = graph.List();
            std::vector<EdgeNode> MST;
            std::priority_queue<EdgeNode, std::vector<EdgeNode>, cmp_EdgeNode> que;
            DSU dsu(n);

            for (int i = 0; i < n; i++) {
                HeadNode node = ver_list[i];
                for (const EdgeNode& e : node.adj) {
                    que.push(e);
                }
            }

            int MST_size = 0;
            while (MST_size != n - 1) {
                EdgeNode e = que.top();
                que.pop();
                if (!dsu.same(e.from, e.dest)) {
                    dsu.unite(e.from, e.dest);
                    MST.push_back(e);
                    MST_size++;
                }
            }

            return MST;//未整理
        }//计算最小生成树，并返回树上的边

        void DFS(LGraph& graph, int current, std::unordered_set<int>& visited, std::vector<int>& route, int& max_interests, int& min_time, std::vector<std::string>& best_route, const std::string& interest_type)
        {
            const auto& vertex = graph.GetVertex(current);
            if (visited.find(current) != visited.end()) {
                return;
            }
            visited.insert(current);
            route.push_back(current);
            int interest_count = 0;
            int total_time = 0;
            for (int idx : route) {
                if (graph.GetVertex(idx).type == interest_type) {
                    interest_count++;
                }
                total_time += graph.GetVertex(idx).visitTime;
            }
            if (interest_count > max_interests || (interest_count == max_interests && total_time < min_time)) {
                max_interests = interest_count;
                min_time = total_time;
                best_route.clear();
                for (int idx : route) {
                    best_route.push_back(graph.GetVertex(idx).name);
                }
            }
            for (const auto& edge : graph.List()[current].adj) {
                DFS(graph, edge.dest, visited, route, max_interests, min_time, best_route, interest_type);
            }
            visited.erase(current);
            route.pop_back();
        }

        std::vector<std::string> PlanTourRouteByInterest(LGraph& graph, const std::string& start, const std::string& interest_type) {
            if (graph.Map().find(start) == graph.Map().end())throw std::exception("起点不存在！");
            
            int start_index = graph.Map().at(start);
            int max_interests = 0;
            int min_time = 0x3f3f3f3f;
            int best_time = 0x3f3f3f3f;
            std::vector<int> route;
            std::unordered_set<int> visited;
            std::vector<std::string> best_route;
            DFS(graph, start_index, visited, route, max_interests, min_time, best_route, interest_type);
            return best_route;
        }

        int PlanTourRouteByTime(LGraph& graph, int time, const std::string& start, std::vector<int>& path) {
            int n = graph.VertexCount();
            const auto& map = graph.Map();
            const auto& verlist = graph.List();

            if (!graph.exist_vertex(start))throw GraphException("该顶点不存在");

            int startID = map.find(start)->second;
            int t0 = graph.GetVertex(start).visitTime;

            if (t0 > time)throw GraphException("你的时间预算太少了，连起点都看不完！");

            std::vector<bool> visited(n, false);
            std::vector<int> currPath;
            std::vector<int> bestPath;
            int bestTime = 0;

            visited[startID] = true;
            currPath.push_back(startID);
            bestTime = t0;
            bestPath = currPath;

            std::function<void(int, int)> dfs = [&](int u, int currTime) {
                // (A) 每次进入，都有可能刷新最佳记录
                if (currTime > bestTime) {
                    bestTime = currTime;
                    bestPath = currPath;
                }

                // (B) 枚举 u 的所有邻接边，尝试继续前进
                for (auto const& edge : verlist[u].adj) {
                    int v = static_cast<int>(edge.dest);
                    if (visited[v]) continue;
                    int travel = edge.weight;                           // u → v 的花费
                    int visitV = graph.GetVertex(v).visitTime;          // v 本身的游览时间
                    int nextTime = currTime + travel + visitV;

                    if (nextTime > time) {
                        // 如果超时，就剪掉，不再深入
                        continue;
                    }

                    // 标记并深入
                    visited[v] = true;
                    currPath.push_back(v);

                    dfs(v, nextTime);

                    // 回溯
                    currPath.pop_back();
                    visited[v] = false;
                }
            };

            dfs(startID, t0);

            path = bestPath;
            return bestTime;
        }
    }
}