//
// Created by 刘凯源 on 24-5-16.
//

#include "LGraph.h"
#include "GraphException.h"

namespace Graph {
    LGraph::LGraph(bool directed)
        : n_verts(0), m_edges(0), directed(directed), ver_list(std::vector<HeadNode>()) {}

    bool LGraph::exist_vertex(const std::string& name) const {
        if (vertex_map.count(name))return true;
        else return false;
    }

    bool LGraph::exist_vertex(const Vertex& id)const {
        if (id >= n_verts)return false;
        else return true;
    }//自己加的

    bool LGraph::exist_edge(const std::string& vertex_x_name, const std::string& vertex_y_name) const {
        if (!exist_vertex(vertex_x_name) || !exist_vertex(vertex_y_name))return false;

        Vertex x_id = vertex_map.find(vertex_x_name)->second;
        Vertex y_id = vertex_map.find(vertex_y_name)->second;
        for (const EdgeNode& e : ver_list[x_id].adj) {
            if (e.dest == y_id)return true;
        }

        return false;
    }

    void LGraph::InsertVertex(const LocationInfo& vertex_info) {
        if (exist_vertex(vertex_info.name))return;
            //throw GraphException("该节点已经存在！");

        HeadNode new_node(vertex_info);
        ver_list.push_back(new_node);
        int new_id = ver_list.size()-1;
        vertex_map[new_node.data.name] = new_id;
        n_verts++;
    }

    void LGraph::DeleteVertex(const LocationInfo& vertex_info) {
        if (!exist_vertex(vertex_info.name))throw GraphException("该节点不存在！");

        Vertex v = vertex_map[vertex_info.name];
        deleted.insert(vertex_info.name);
        vertex_map.erase(vertex_info.name);
        ver_list[v].adj.clear();
        ver_list.erase(ver_list.begin() + v);

        for (auto it = ver_list.begin() + v; it != ver_list.end(); it++) {
            vertex_map[it->data.name]--;
        }//更改map


        for (auto& head : ver_list)
        {
            for (auto it = head.adj.begin(); it != head.adj.end();it++)
            {
                if (it->from > v)it->from--;
                if (it->dest == v)
                {
                    it = head.adj.erase(it);
                    break;
                }
                else if (it->dest > v)it->dest--;
            }
        }//更改verlist
        n_verts--;
    }//删除节点，改变邻接表和map，放入deleted，并减少顶点数，别的函数无需考虑虚拟节点

    void LGraph::UpdateVertex(const LocationInfo& old_info, LocationInfo& new_info) {
        if (!exist_vertex(old_info.name))throw GraphException("该节点不存在");

        int id = vertex_map[old_info.name];
        ver_list[id].data = new_info;
    }//更新节点，新/旧节点的信息由LocationInfo类给出

    VertInfo LGraph::GetVertex(const std::string& name) const {
        if (!exist_vertex(name))throw GraphException("不存在该节点！");

        Vertex id = vertex_map.find(name)->second;
        return ver_list[id].data;
    }//获取节点，由节点名查询节点信息

    VertInfo LGraph::GetVertex(const Vertex vertex) const {
        if (vertex >= ver_list.size())throw GraphException("不存在该节点！");//不对用户开放
        
        return ver_list[vertex].data;
    }//获取节点，由节点的ID查询

    void LGraph::InsertEdge(const std::string& vertex_x_name, const std::string& vertex_y_name, GElemSet weight) {
        if (!exist_vertex(vertex_x_name) || !exist_vertex(vertex_y_name))return;
            //throw GraphException("节点不存在！");
        if (exist_edge(vertex_x_name, vertex_y_name))return;
            //throw GraphException("该边已经存在！");
        
        Vertex x_id = vertex_map[vertex_x_name];
        Vertex y_id = vertex_map[vertex_y_name];

        ver_list[x_id].adj.push_back(EdgeNode(x_id, y_id, weight));
        m_edges++;
        if (directed == false) {
            ver_list[y_id].adj.push_back(EdgeNode(y_id, x_id, weight));
        }
    }//插入边

    void LGraph::DeleteEdge(const std::string& vertex_x_name, const std::string& vertex_y_name) {
        if (!exist_edge(vertex_x_name, vertex_y_name))throw GraphException("该边不存在！");

        Vertex x_id = vertex_map[vertex_x_name];
        Vertex y_id = vertex_map[vertex_y_name];

        for (auto it = ver_list[x_id].adj.begin(); it != ver_list[x_id].adj.end();it++) {
            if (it->dest == y_id) {
                ver_list[x_id].adj.erase(it);
                break;
            }
        }
        m_edges--;

        if (directed == false) {
            for (auto it = ver_list[y_id].adj.begin(); it != ver_list[y_id].adj.end(); it++) {
                if (it->dest == x_id) {
                    ver_list[y_id].adj.erase(it);
                    break;
                }
            }
        }
    }//删除边，由两个节点名确定一条边

    void LGraph::DeleteEdge(Vertex vertex_x, Vertex vertex_y) {
        if (!exist_edge(ver_list[vertex_x].data.name, ver_list[vertex_y].data.name))throw GraphException("该边不存在！");

        VertInfo x = GetVertex(vertex_x);
        VertInfo y = GetVertex(vertex_y);

        if (!exist_edge(x.name, y.name))return;//

        Vertex x_id = vertex_map[x.name];
        Vertex y_id = vertex_map[y.name];

        for (auto it = ver_list[x_id].adj.begin(); it != ver_list[x_id].adj.end(); it++) {
            if (it->dest == y_id) {
                ver_list[x_id].adj.erase(it);
                break;
            }
        }
        m_edges--;

        if (directed == false) {
            for (auto it = ver_list[y_id].adj.begin(); it != ver_list[y_id].adj.end(); it++) {
                if (it->dest == x_id) {
                    ver_list[y_id].adj.erase(it);
                    break;
                }
            }
        }
    }//删除边，由两个节点ID确定一条边

    void LGraph::UpdateEdge(const std::string& vertex_x_name, const std::string& vertex_y_name, GElemSet new_weight) {
        if (!exist_edge(vertex_x_name, vertex_y_name))throw GraphException("该边不存在！");//出错

        Vertex x_id = vertex_map[vertex_x_name];
        Vertex y_id = vertex_map[vertex_y_name];

        for (EdgeNode& e : ver_list[x_id].adj) {
            if (e.dest == y_id) {
                e.weight = new_weight;
                break;
            }
        }

        if (directed == false) {
            for (EdgeNode& e : ver_list[y_id].adj) {
                if (e.dest == x_id) {
                    e.weight = new_weight;
                    break;
                }
            }
        }
    }//更新边，由两个节点名确定一条边

    GElemSet LGraph::GetEdge(const std::string& vertex_x_name, const std::string& vertex_y_name) const {
        if (!exist_edge(vertex_x_name,vertex_y_name))throw GraphException("该边不存在！");//出错

        Vertex x_id = vertex_map.find(vertex_x_name)->second;
        Vertex y_id = vertex_map.find(vertex_y_name)->second;
        for (const EdgeNode& e : ver_list[x_id].adj) {
            if (e.dest == y_id)return e.weight;
        }

        return GElemSet();//理论上不会到这一步
    }//获取边的信息

    std::vector<EdgeNode> LGraph::SortedEdges(std::function<bool(const GElemSet&, const GElemSet&)> cmp) const{ 
        std::vector<EdgeNode> vec;
        for (const HeadNode& node : ver_list) {
            for (const EdgeNode& e : node.adj) {
                if (e.from < e.dest)vec.push_back(e);
            }
        }

        if (cmp) {
            // 有 cmp：按权重排序
            std::sort(vec.begin(), vec.end(),
                [&](const EdgeNode& a, const EdgeNode& b) {
                    return cmp(a.weight, b.weight);
                }
            );
        }
        else {
            // 无 cmp：先 by.from，再 by.to
            std::sort(vec.begin(), vec.end(),
                [](const EdgeNode& a, const EdgeNode& b) {
                    if (a.from != b.from) return a.from < b.from;
                    return a.dest < b.dest;
                }
            );
        }

        return vec;
    }//获取按边权和给定规则排序的所有边
}