#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <cassert>
#include "LGraph.h"
#include "Algorithm.h"
#include "LocationInfo.h"
#include "GraphException.h"

using Graph::LGraph;
using Graph::VertInfo;
using namespace Graph::Algorithm;

// Hint: 如果不慎写坏项目附带的两个文件，可以在backup文件夹中获取原始文件
// TODO: 请根据可执行文件和csv的位置填写文件路径

//const std::string nodes_path{ "nodes.csv" };
//const std::string edges_path{ "edges.csv" };

//const std::string nodes_path{ "nodes_small_test.csv" };
//const std::string edges_path{ "edges1_small_test.csv" };

//const std::string nodes_path{ "Data1/nodes_chain_5.csv" };
//const std::string edges_path{ "Data1/edges_chain_5.csv" };

const std::string nodes_path{ "cases/star_100/nodes.csv" };
const std::string edges_path{ "cases/star_100/edges.csv" };

struct edge {
    std::string from;
    std::string to;
    int length;
};

void init(LGraph& graph);

std::vector<VertInfo> read_nodes(const std::string& nodes_path);

std::vector<edge> read_edges(const std::string& edges_path);

void store_nodes(const std::string& nodes_path, const LGraph& graph);

void show_all_types(const LGraph& graph);

void show_specific_type(const LGraph& graph, std::string);

void store_edges(const std::string& edges_path, const LGraph& graph);

void show_all_nodes(const LGraph& graph);

void show_all_edges(const LGraph& graph);

int main() {

    LGraph graph(false); //初始化一个无向图。
    init(graph);
    while (true) {
        int choice{ 0 };
        std::cout << "欢迎使用校园导航系统！" << std::endl
            << "请选择您要进行的操作：" << std::endl
            //<< "(如果读入中文文件请确保使用ANSI/GBK编码)" << std::endl
            << "1.顶点相关操作" << std::endl
            << "2.边相关操作" << std::endl
            << "3.从文件中重新加载点与边" << std::endl
            << "4.是否连通" << std::endl
            << "5.是否存在欧拉通路" << std::endl
            << "6.求任意两点间的最短距离" << std::endl
            << "7.求最小生成树" << std::endl
            << "8.求解拓扑受限时的最短路径" << std::endl
            << "9.根据景点类型推荐游览路线" << std::endl
            << "10.根据时间预算推荐游览路线" << std::endl
            << "11.退出程序" << std::endl
            << "请输入操作前的数字：";
        std::cin >> choice;
        if (choice == 1) { // 顶点相关操作
            std::cout << "顶点相关操作：" << std::endl
                << "1.根据名字输出顶点信息" << std::endl
                << "2.根据类别输出顶点信息" << std::endl
                << "3.输出所有顶点信息" << std::endl
                << "4.添加一个顶点" << std::endl
                << "5.删除一个顶点" << std::endl
                << "6.更新一个顶点" << std::endl
                << "7.输出顶点相连的边" << std::endl
                << "8.存储顶点到文件中" << std::endl
                << "9.回到上一级菜单" << std::endl
                << "请输入操作前的数字：";
            std::cin >> choice;
            if (choice == 1) {
                std::string name;
                std::cout << "请输入顶点名称：";
                std::cin >> name;
                try {
                    VertInfo v{ graph.GetVertex(name) };
                    std::cout << "顶点名称：" << v.name << std::endl
                        << "顶点类型：" << v.type << std::endl
                        << "建议游览时间：" << v.visitTime << std::endl;

                }
                catch (Graph::GraphException& e) {
                    std::cout << "名称为 " << name << " 的顶点不存在" << std::endl;
                }
            }
            else if (choice == 2) {
                std::cout << "请输入你想查询的类别" << std::endl;
                std::string t;
                std::cin >> t;
                std::vector<std::string> vec;
                for (auto& it : graph.List()) {
                    if (it.data.type == t)vec.push_back(it.data.name);
                }
                try {
                    if (vec.empty())throw std::exception("不存在该类别");

                    for (auto& it : vec) {
                        std::cout << it << ' ';
                    }
                    std::cout << std::endl;
                }
                catch (std::exception& e) {
                    std::cout << e.what() << std::endl;
                }
                
            }
            else if (choice == 3) {
                std::cout << "顶点名称, 顶点类型, 建议游览时间：" << std::endl;
                show_all_nodes(graph);
            }
            else if (choice == 4) {
                std::cout << "请输入顶点名称，顶点类型，建议游览时间(空格分隔)：";
                std::string name, type;
                int visit_time;
                std::cin >> name >> type >> visit_time;
                try {
                    graph.InsertVertex({ name, visit_time, type });
                }
                catch (Graph::GraphException& e) {
                    std::cout << e.what() << std::endl;
                }
            }
            else if (choice == 5) {
                std::cout << "请输入顶点名称：";
                std::string name;
                std::cin >> name;
                try {
                    graph.DeleteVertex({ name, 0, "" });
                }
                catch (Graph::GraphException& e) {
                    std::cout << e.what() << std::endl;
                }
            }
            else if (choice == 6) {
                std::cout << "请输入原来的顶点名称,更新后的顶点名称，顶点类型，建议游览时间(空格分隔)";
                std::string old_name, new_name, node_type;
                int visit_time;
                std::cin >> old_name >> new_name >> node_type >> visit_time;
                VertInfo new_node(new_name, visit_time, node_type);
                try {
                    graph.UpdateVertex(graph.GetVertex(old_name), new_node);
                }
                catch (Graph::GraphException& e) {
                    std::cout << e.what() << std::endl;
                }
            }
            else if (choice == 7) {
                std::string s;
                std::cout << "请输入你想查询的顶点" << std::endl;
                std::cin >> s;

                try {
                    if (!graph.exist_vertex(s))throw std::exception("该顶点不存在！");
                    auto vec = graph.List()[graph.Map().find(s)->second].adj;
                    if (vec.empty()) {
                        std::cout << "该点无边" << std::endl;
                        break;
                    }
                    for (auto& it : vec) {
                        std::cout << graph.List()[it.dest].data.name << '(' << it.weight << ')' << ' ';
                    }
                    std::cout << std::endl;
                }
                catch (std::exception& e) {
                    std::cout << e.what() << std::endl;
                }
                
            }
            else if (choice == 8) {
                std::cout << "正在将顶点存储到test_nodes.csv中..." << std::endl;
                store_nodes("test_nodes.csv", graph);
                std::cout << "存储成功！" << std::endl;
            }
        }
        else if (choice == 2) { // 边相关操作
            std::cout << "边相关操作：" << std::endl
                << "1.输出特定边信息" << std::endl
                << "2.输出所有边信息" << std::endl
                << "3.添加一条边" << std::endl
                << "4.删除一条边" << std::endl
                << "5.更新一条边" << std::endl
                << "6.存储边到文件中" << std::endl
                << "7.回到上一级菜单" << std::endl
                << "请输入操作前的数字：";
            std::cin >> choice;
            if (choice == 1) {
                std::string from, to;
                std::cout << "请输入边的起点与终点(空格分隔)：";
                std::cin >> from >> to;
                try {
                    Graph::GElemSet w{ graph.GetEdge(from, to) };
                    std::cout << from << " <---> " << to << " 距离为：" << w << std::endl;
                }
                catch (Graph::GraphException& e) {
                    std::cout << from << " <---> " << to << " 不存在！" << std::endl;
                }
            }
            else if (choice == 2) {
                show_all_edges(graph);
            }
            else if (choice == 3) {
                std::cout << "请输入起点名称，终点名称，距离(空格分隔)：";
                std::string from, to;
                int length;
                std::cin >> from >> to >> length;
                try {
                    graph.InsertEdge(from, to, length);
                }
                catch (Graph::GraphException& e) {
                    std::cout << e.what() << std::endl;
                }

            }
            else if (choice == 4) {
                std::cout << "请输入起点名称，终点名称(空格分隔)：";
                std::string from, to;
                std::cin >> from >> to;
                try {
                    graph.DeleteEdge(from, to); // 删除一条边
                }
                catch (Graph::GraphException& e) {
                    std::cout << e.what() << std::endl;
                }

            }
            else if (choice == 5) {
                std::cout << "请输入起点名称，终点名称，距离(空格分隔)：";
                std::string from, to;
                int length;
                std::cin >> from >> to >> length;
                try {
                    graph.UpdateEdge(from, to, length);
                }
                catch (Graph::GraphException& e) {
                    std::cout << e.what() << std::endl;
                }
            }
            else if (choice == 6) {
                std::cout << "正在将边存储到test_edges.csv文件中..." << std::endl;
                store_edges("test_edges.csv", graph);
                std::cout << "存储成功！" << std::endl;
            }
        }
        else if (choice == 3) {// 从文件中重新加载点与边
            init(graph);
        }
        else if (choice == 4) {
            auto res = IsConnected(graph);
            std::cout << (res ? "该图连通" : "该图不连通") << std::endl;
        }
        else if (choice == 5) { // 是否存在欧拉通路
            auto res = HaveEulerCircle(graph);
            std::cout << (res ? "存在欧拉通路" : "不存在欧拉通路") << std::endl;
        }
        else if (choice == 6) { // 求任意两点间的最短距离
            std::cout << "请输入两个地点，使用空格分开：" << std::endl;
            std::string x, y;
            std::vector<int> pre;
            std::cin >> x >> y;
            try {
                std::cout << x << "和" << y << "之间的最短距离为：" << GetShortestPath(graph, pre, x, y) << std::endl;
                int end = graph.Map().find(y)->second;
                std::stack<std::string> s;
                for (int x = end; pre[x] != x; x = pre[x]) {
                    s.push(graph.List()[x].data.name);
                }
                s.push(x);
                std::cout << "路径为: ";
                while (!s.empty()) {
                    std::cout << s.top() << ' ';
                    s.pop();
                }
                std::cout << std::endl;
            }
            catch (Graph::GraphException& e) {
                std::cout << "你找到了虚空的距离" << std::endl;
            }
        }
        else if (choice == 7) { // 求最小生成树
            if (IsConnected(graph)) {
                std::cout << "最小生成树的点如下：";
                auto res = MinimumSpanningTree(graph);
                int sum = 0;
                for (const auto& item : res) {//
                    const auto& e = edge{ graph.List()[item.from].data.name, graph.List()[item.dest].data.name, item.weight };
                    std::cout << e.from << "," << e.to << "," << e.length << std::endl;
                    sum += e.length;
                }
                std::cout << "总权重为" << sum << std::endl;
            }
            else {
                std::cout << "图不连通" << std::endl;
            }
        }
        else if (choice == 8) { // 求解拓扑受限时的最短路径
            std::cout << "请输入您希望的拓扑序，第一行一个n为序列长度，第二行n个地点为拓扑序列" << std::endl;
            int n;
            std::vector<std::string> list;
            std::cin >> n;
            while (n--) {
                std::string x;
                std::cin >> x;
                list.push_back(x);
            }
            std::cout << "最短路径为" << TopologicalShortestPath(graph, list) << std::endl;
        }
        else if (choice == 9) {
            show_all_types(graph);
            std::cout << "请输入感兴趣的景点类型：";
            std::string interest;
            std::cin >> interest;

            
            try {
                show_specific_type(graph, interest);
                std::cout << "请输入起点：";
                std::string start;
                std::cin >> start;

                std::vector<std::string> route = PlanTourRouteByInterest(graph, start, interest);
                if (route.size() == 1) {
                    std::cout << "很抱歉，该景点周围暂无同类型景点！" << std::endl;
                }
                else {
                    std::cout << "推荐的游览路线是：" << std::endl;
                    bool is_first = true;
                    for (const auto& place : route) {
                        if (is_first)
                        {
                            std::cout << place;
                            is_first = false;
                        }
                        else std::cout << "---->" << place;

                    }
                    std::cout << std::endl;
                }
            }
            catch (const std::exception& e) {
                std::cout << e.what() << std::endl;
            }
        }
        else if (choice == 10) {
            show_all_nodes(graph);
            std::cout << "请输入您的时间预算和起点：";
            int time;
            std::string start;
            std::cin >> time >> start;

            try {
                std::vector<int> path;
                int totaltime = PlanTourRouteByTime(graph, time, start, path);
                std::cout << "推荐的游览路线是：" << std::endl;

                bool is_first = true;
                for (const auto& it : path) {
                    if (is_first) {
                        std::cout << graph.List()[it].data.name;
                        is_first = false;
                    }
                    else std::cout << "---->" << graph.List()[it].data.name ;
                }
                std::cout << std::endl;
            }
            catch (Graph::GraphException& e) {
                std::cout << e.what() << std::endl;
            }
        }
        else {
            std::cout << "感谢您的使用，再见！\n 开发者：\\lky1433223/ \\Voltline/" << std::endl;
            break;
        }
    }
        
    return 0;
}

/* 从文件中读取顶点信息
 * @param nodes_path: 顶点文件路径
 * @return: 包含读取到的所有顶点的std::vector
 */
std::vector<VertInfo> read_nodes(const std::string& nodes_path) {
    std::ifstream fin(nodes_path);              // 创建文件输入流对象
    assert(fin.good());                         // 断言，通过文件输入流的good()方法检查文件是否存在
    std::vector<VertInfo> nodes;                // 临时存储顶点信息
    std::string line;                           // 通过std::getline读取到的每一行，临时存储在字符串line中
    while (std::getline(fin, line)) {       // 通过std::getline(std::istream&, std::string&)来获取输入流的一行
        for (auto& c : line) {
            if (c == ','||c=='\"') c = ' ';              // csv文件以逗号分隔，为了方便后续使用字符串输入流，将逗号替换为空格
        }
        if (line.substr(0, 4) == "name")continue;

        std::istringstream sin(line);           // 利用std::istringstream，以获取的行为内容构建一个字符串输入流
        std::string name, type;                 // 文件每行中的顶点名，顶点类型
        int visit_time;                         // 文件中每行的顶点推荐游览时间
        sin >> name >> type >> visit_time;      // 利用已构造的字符串输入流，获取对应的几个参数
        nodes.push_back({ name, visit_time, type });  // 放入需要返回的std::vector中
    }
    fin.close();                                // 文件使用完毕后，请务必使用close访问关闭输入流，避免文件损坏
    return nodes;
}

/* 从文件中读取边信息
 * @param edges_path: 边文件路径
 * @return: 包含读取到的所有边的std::vector
 */
std::vector<edge> read_edges(const std::string& edges_path) {
    // 请参考read_nodes函数的内容与提示，结合edges.csv文件的内容，完成本函数的定义
    std::ifstream fin(edges_path);
    assert(fin.good());
    std::vector<edge> edges;
    std::string line;
    while (std::getline(fin, line)) {
        for (auto& c : line) {
            if (c == ','||c=='\"')c = ' ';
        }
        if (line.substr(0, 4) == "node")continue;

        std::istringstream sin(line);
        std::string from, to;
        int length;
        sin >> from >> to >> length;
        edges.push_back({ from,to,length });
    }
    fin.close();
    return edges;
}

void init(LGraph& graph) {
    std::vector<VertInfo> nodes{ read_nodes(nodes_path) };        // 通过read_nodes函数获得顶点信息
    std::vector<edge> edges{ read_edges(edges_path) };            // 通过read_edges函数获得边信息
    for (auto& v : nodes) {
        graph.InsertVertex(v);
    }//向图graph中插入顶点
    for (auto& e : edges) {
        graph.InsertEdge(e.from, e.to, e.length);
    }//向图graph中插入边
}

/* 打印目前的所有顶点
 * @param graph: 图
 */
void show_all_nodes(const LGraph& graph) {
    std::cout << std::left << std::setw(55) << "景点名称" << std::setw(40) << "景点类型" << std::setw(8) << "游览时间" << std::endl;
    //std::cout << "景点名称,景点类型,游览时间" << std::endl;
    for (const Graph::HeadNode& node : graph.List()) {
        std::cout << std::setw(55) << node.data.name << std::setw(40) << node.data.type << std::setw(8) << node.data.visitTime << std::endl;
        //std::cout << node.data.name << "," << node.data.type << ',' << node.data.visitTime << std::endl;
    }
}// 实现打印所有顶点信息
 // 每行输出格式：顶点名,顶点类型,推荐游览时间

void show_all_types(const LGraph& graph) {
    std::set<std::string> types;
    for (const auto& head : graph.List()) {
        types.insert(head.data.type);
    }
    std::cout << "所有景点类型：" << std::endl;
    for (const auto& type : types) {
        std::cout << type << std::endl;
    }
}

void show_specific_type(const LGraph& graph,std::string type) {
    std::set<std::string> locations;
    for (const auto& it : graph.List()) {
        if (it.data.type == type)locations.insert(it.data.name);
    }

    if (locations.size() == 0)throw std::exception("该类型不存在");

    std::cout << "所有类型为" << type << "的景点为：" << std::endl;
    for (const auto& l : locations) {
        std::cout << l << std::endl;
    }
}

/* 打印目前的所有边
 * @param graph: 图
 */
void show_all_edges(const LGraph& graph) {
    // LGraph类只提供了获取邻接表和获取经过排序的边的方法
    // 为了能够获取到每条边的元信息，我们需要配合两组数据来完成信息的获取
    const auto& graph_edges{ graph.SortedEdges() };       // 获取经过排序的边
    const auto& graph_list{ graph.List() };               // 获取邻接表
    std::cout << std::left << std::setw(55) << "起点" << std::setw(40) << "终点" << std::setw(8) << "距离" << std::endl;
    for (const auto& en : graph_edges) {                 // 遍历边，通过下标来获取对应的顶点信息
        const auto& e = edge{ graph_list[en.from].data.name, graph_list[en.dest].data.name, en.weight };
        std::cout << std::left << std::setw(55) << e.from << std::setw(40) << e.to << std::setw(8) << e.length << std::endl;
    }
}

/* 将顶点信息存储回文件中
 * @param nodes_path: 顶点文件路径
 * @param graph: 图
 */
void store_nodes(const std::string& nodes_path, const LGraph& graph) {
    std::ofstream fout(nodes_path);                                         // 通过目录创建文件输出流
    const auto& graph_list{ graph.List() };                                   // 通过LGraph对象的List()方法获得邻接表
    for (const auto& n : graph_list) {                                       // 遍历邻接表中的每个顶点
        const auto& v = n.data;                                             // 获得顶点中的元数据
        fout << v.name << "," << v.type << "," << v.visitTime << std::endl; // 按照顶点存储的格式，重新输入到文件中
    }
    fout.close();                                                           // 在使用完任何文件流后，都要使用close方法关闭
}

/* 将边信息存储回文件中
 * @param edges_path: 边文件路径
 * @param graph: 图
 */
void store_edges(const std::string& edges_path, const LGraph& graph) {
    std::ofstream fout(edges_path);
    const auto& graph_edges{ graph.SortedEdges() };       // 获取经过排序的边
    const auto& graph_list{ graph.List() };               // 获取邻接表
    for (const auto& en : graph_edges) {                 // 遍历边，通过下标来获取对应的顶点信息
        const auto& e = edge{ graph_list[en.from].data.name, graph_list[en.dest].data.name, en.weight };
        fout << e.from << "," << e.to << "," << e.length << std::endl;
    }
    fout.close();
}    //请参考store_nodes函数的内容与提示，结合edges.csv文件的内容
    //以及show_all_edges函数中获取边的元信息的方式完成store_edges函数的定义