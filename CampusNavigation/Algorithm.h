//
// Created by 刘凯源 on 24-10-13.
//

#ifndef CAMPUSNAVIGATION_ALGORITHM_H
#define CAMPUSNAVIGATION_ALGORITHM_H

#include "LGraph.h"

namespace Graph {
    namespace Algorithm {
        class DSU {
        private:
            std::vector<int> parent, rank;
        public:
            explicit DSU(int n) {
                parent.resize(n);
                for (int i = 0; i < n; i++) {
                    parent[i] = i;
                }
                rank.assign(n, 0);
            }// 请完成并查集的构造函数

            int find(int x){
                if (x >= parent.size() || x < 0)return NIL;//不存在

                while (parent[x] != x) {
                    x = parent[x];
                }
                return x;
            }//查找元素的根节点，并完成路径压缩。其实没有压缩

            void unite(int x, int y) {
                if (x >= parent.size() || x < 0 || y >= parent.size() || y < 0)return;//不存在

                int x_root = find(x);
                int y_root = find(y);

                if (rank[x_root] > rank[y_root])parent[y_root] = x_root;
                else if (rank[x_root] < rank[y_root])parent[x_root] = y_root;
                else {
                    parent[x_root] = y_root;
                    rank[y_root]++;
                }
            }// 合并集合

            bool same(int x, int y) const {
                if (x >= parent.size() || x < 0 || y >= parent.size() || y < 0)return false;//不存在

                while (parent[x] != x) {
                    x = parent[x];
                }
                while (parent[y] != y) {
                    y = parent[y];
                }

                return x == y;
            }//检查两个节点是否属于同一个集合
        };

        std::list<Vertex> EulerCircle(const LGraph& graph); //计算欧拉回路
        bool HaveEulerCircle(const LGraph& graph); //判断是否存在欧拉回路
        bool IsConnected(const LGraph& graph); //判断图是否联通
        int GetShortestPath(const LGraph& graph, std::vector<int>& pre, const std::string& vertex_name_x,
        const std::string& vertex_name_y); //计算单源最短路径
        int TopologicalShortestPath(const LGraph& graph, std::vector<std::string> path); //计算拓扑受限的最短路径
        std::vector<EdgeNode> MinimumSpanningTree(const LGraph& graph); //计算最小生成树

        std::vector<std::string> PlanTourRouteByInterest(LGraph& graph, const std::string& start, const std::string& interest_type);
        int PlanTourRouteByTime(LGraph& graph, int time, const std::string& start, std::vector<int>& path);
    }
}
#endif //CAMPUSNAVIGATION_ALGORITHM_H
