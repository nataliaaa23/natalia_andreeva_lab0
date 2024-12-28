#include <iostream>
#include <vector>
#include <list>
#include <queue>
#include <map>
#include <sstream>
#include <fstream>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stack>
#include <iomanip>
#include <utility>
#include <set>

using namespace std;

class Graph {
public:
    map<pair<double, double>, list<pair<pair<double, double>, double> > > adj_list;

    void add_edge(double lon1, double lat1, double lon2, double lat2, double weight) {
        adj_list[{lon1, lat1}].push_back({{lon2, lat2}, weight});
        adj_list[{lon2, lat2}].push_back({{lon1, lat1}, weight});
    }

    pair<double, double> find_closest_node(double lon, double lat) {
        pair<double, double> closest_node;
        double min_distance = numeric_limits<double>::infinity();
        for (const auto &node: adj_list) {
            double distance = sqrt(pow(node.first.first - lon, 2) + pow(node.first.second - lat, 2));
            if (distance < min_distance) {
                min_distance = distance;
                closest_node = node.first;
            }
        }
        return closest_node;
    }

    vector<pair<double, double> > dijkstra(pair<double, double> start, pair<double, double> end) {
        pair<double, double> closest_start = find_closest_node(start.first, start.second);
        pair<double, double> closest_end = find_closest_node(end.first, end.second);

        map<pair<double, double>, double> distances;
        map<pair<double, double>, pair<double, double> > previous;
        priority_queue<pair<double, pair<double, double> >, vector<pair<double, pair<double, double> > >, greater<> >
                pq;

        for (const auto &node: adj_list) {
            distances[node.first] = numeric_limits<double>::infinity();
        }
        distances[closest_start] = 0;
        pq.push({0, closest_start});

        while (!pq.empty()) {
            auto current = pq.top().second;
            pq.pop();

            if (current == closest_end) break;

            for (const auto &neighbor: adj_list[current]) {
                double new_dist = distances[current] + neighbor.second;
                if (new_dist < distances[neighbor.first]) {
                    distances[neighbor.first] = new_dist;
                    previous[neighbor.first] = current;
                    pq.push({new_dist, neighbor.first});
                }
            }
        }

        vector<pair<double, double> > path;
        if (previous.find(closest_end) != previous.end() || closest_start == closest_end) {
            for (auto at = closest_end; at != closest_start; at = previous[at]) {
                path.push_back(at);
            }
            path.push_back(closest_start);
            reverse(path.begin(), path.end());
        }

        return path;
    }

    vector<pair<double, double> > bfs(pair<double, double> start, pair<double, double> end) {
        pair<double, double> closest_start = find_closest_node(start.first, start.second);
        pair<double, double> closest_end = find_closest_node(end.first, end.second);

        map<pair<double, double>, bool> visited;
        map<pair<double, double>, pair<double, double> > previous;
        queue<pair<double, double> > q;

        visited[closest_start] = true;
        q.push(closest_start);

        while (!q.empty()) {
            auto current = q.front();
            q.pop();

            if (current == closest_end) break;

            for (const auto &neighbor: adj_list[current]) {
                if (!visited[neighbor.first]) {
                    visited[neighbor.first] = true;
                    previous[neighbor.first] = current;
                    q.push(neighbor.first);
                }
            }
        }

        vector<pair<double, double> > path;
        for (auto at = closest_end; at != closest_start; at = previous[at]) {
            path.push_back(at);
        }
        path.push_back(closest_start);
        reverse(path.begin(), path.end());

        return path;
    }

    vector<pair<double, double> > dfs(pair<double, double> start, pair<double, double> end) {
        pair<double, double> closest_start = find_closest_node(start.first, start.second);
        pair<double, double> closest_end = find_closest_node(end.first, end.second);

        map<pair<double, double>, bool> visited;
        map<pair<double, double>, pair<double, double> > parent;
        stack<pair<double, double> > s;

        s.push(closest_start);
        visited[closest_start] = true;

        while (!s.empty()) {
            auto current = s.top();
            s.pop();

            if (current == closest_end) {
                break;
            }

            for (const auto &neighbor: adj_list[current]) {
                if (!visited[neighbor.first]) {
                    visited[neighbor.first] = true;
                    parent[neighbor.first] = current;
                    s.push(neighbor.first);
                }
            }
        }

        vector<pair<double, double> > path;
        if (visited[closest_end]) {
            for (auto at = closest_end; at != closest_start; at = parent[at]) {
                path.push_back(at);
            }
            path.push_back(closest_start);
            reverse(path.begin(), path.end());
        }

        return path;
    }

    static double heuristic(pair<double, double> a, pair<double, double> b) {
        return sqrt(pow(a.first - b.first, 2) + pow(a.second - b.second, 2));
    }

    vector<pair<double, double> > a_star(pair<double, double> start, pair<double, double> end) {
        pair<double, double> closest_start = find_closest_node(start.first, start.second);
        pair<double, double> closest_end = find_closest_node(end.first, end.second);

        map<pair<double, double>, double> g_score;
        map<pair<double, double>, double> f_score;
        priority_queue<pair<double, pair<double, double> >, vector<pair<double, pair<double, double> > >, greater<> >
                open_set;

        g_score[closest_start] = 0;
        f_score[closest_start] = heuristic(closest_start, closest_end);
        open_set.push({f_score[closest_start], closest_start});

        map<pair<double, double>, pair<double, double> > came_from;

        while (!open_set.empty()) {
            auto current = open_set.top().second;
            open_set.pop();

            if (current == closest_end) {
                vector<pair<double, double> > path;
                for (auto at = closest_end; at != closest_start; at = came_from[at]) {
                    path.push_back(at);
                }
                path.push_back(closest_start);
                reverse(path.begin(), path.end());
                return path;
            }

            for (const auto &neighbor: adj_list[current]) {
                double tentative_g_score = g_score[current] + neighbor.second;
                if (g_score.find(neighbor.first) == g_score.end() || tentative_g_score < g_score[neighbor.first]) {
                    came_from[neighbor.first] = current;
                    g_score[neighbor.first] = tentative_g_score;
                    f_score[neighbor.first] = g_score[neighbor.first] + heuristic(neighbor.first, closest_end);
                    open_set.push({f_score[neighbor.first], neighbor.first});
                }
            }
        }

        return {};
    }

    void load_graph_from_file(const string &filename) {
        ifstream file(filename);
        if (!file.is_open()) {
            cerr << "Error opening file: " << filename << endl;
            return;
        }

        string line;
        while (getline(file, line)) {
            stringstream line_stream(line);
            string node_part;
            getline(line_stream, node_part, ':');
            stringstream node_stream(node_part);
            string lon_lat_str;
            getline(node_stream, lon_lat_str, ',');
            double lon1 = stod(lon_lat_str);
            getline(node_stream, lon_lat_str);
            double lat1 = stod(lon_lat_str);

            string edge_part;
            while (getline(line_stream, edge_part, ';')) {
                stringstream edge_stream(edge_part);
                string lon_lat_weight_str;
                getline(edge_stream, lon_lat_weight_str, ',');
                double lon2 = stod(lon_lat_weight_str);
                getline(edge_stream >> ws, lon_lat_weight_str);
                double lat2 = stod(lon_lat_weight_str);
                getline(edge_stream >> ws, lon_lat_weight_str);
                double weight = stod(lon_lat_weight_str);

                add_edge(lon1, lat1, lon2, lat2, weight);
            }
        }
        file.close();
    }
};

void measure_dijkstra(Graph &g, pair<double, double> start, pair<double, double> end) {
    auto start_time = chrono::high_resolution_clock::now();
    auto path = g.dijkstra(start, end);
    auto end_time = chrono::high_resolution_clock::now();

    double elapsed_time = chrono::duration_cast<chrono::microseconds>(end_time - start_time).count() / 1000000.0;
    cout << fixed << setprecision(6);
    cout << "Dijkstra's algorithm execution time: " << elapsed_time << " seconds" << endl;
}

void measure_a_star(Graph &g, pair<double, double> start, pair<double, double> end) {
    auto start_time = chrono::high_resolution_clock::now();
    auto path_a_star = g.a_star(start, end);
    auto end_time = chrono::high_resolution_clock::now();

    double elapsed_time = chrono::duration_cast<chrono::microseconds>(end_time - start_time).count() / 1000000.0;
    cout << fixed << setprecision(6);
    cout << "A* algorithm execution time: " << elapsed_time << " seconds" << endl;
}

void measure_bfs(Graph &g, pair<double, double> start, pair<double, double> end) {
    auto start_time = chrono::high_resolution_clock::now();
    auto bfs_path = g.bfs(start, end);
    auto end_time = chrono::high_resolution_clock::now();

    double elapsed_time = chrono::duration_cast<chrono::microseconds>(end_time - start_time).count() / 1000000.0;
    cout << fixed << setprecision(6);
    cout << "BFS execution time: " << elapsed_time << " seconds" << endl;
}

void measure_dfs(Graph &g, pair<double, double> start, pair<double, double> end) {
    auto start_time = chrono::high_resolution_clock::now();
    auto dfs_path = g.dfs(start, end);
    auto end_time = chrono::high_resolution_clock::now();

    double elapsed_time = chrono::duration_cast<chrono::microseconds>(end_time - start_time).count() / 1000000.0;
    cout << fixed << setprecision(6);
    cout << "DFS execution time: " << elapsed_time << " seconds" << endl;
}

int main() {
    Graph test_graph;
    test_graph.add_edge(1.0, 0.0, 2.0, 0.0, 5); // (1,2,5)
    test_graph.add_edge(2.0, 0.0, 3.0, 0.0, 6); // (2,3,6)
    test_graph.add_edge(2.0, 0.0, 4.0, 0.0, 11); // (2,4,11)
    test_graph.add_edge(3.0, 0.0, 5.0, 0.0, 12); // (3,5,12)
    test_graph.add_edge(3.0, 0.0, 4.0, 0.0, 2); // (3,4,2)
    test_graph.add_edge(4.0, 0.0, 5.0, 0.0, 10); // (4,5,10)
    test_graph.add_edge(5.0, 0.0, 6.0, 0.0, 10); // (4,6,10)
    //test_graph.add_edge(5.0, 0.0, 1.0, 0.0, 180); // (4,6,18)

    cout << "Dijkstra's Algorithm:" << endl;
    auto dijkstra_path = test_graph.dijkstra({1, 0}, {6, 0});
    cout << "Path found by Dijkstra:  ";
    double total_weight = 0;
    for (size_t i = 0; i < dijkstra_path.size(); ++i) {
        cout << "(" << dijkstra_path[i].first << "," << dijkstra_path[i].second << ") ";
        if (i < dijkstra_path.size() - 1) {
            for (const auto &neighbor: test_graph.adj_list[dijkstra_path[i]]) {
                if (neighbor.first == dijkstra_path[i + 1]) {
                    total_weight += neighbor.second;
                    break;
                }
            }
        }
    }
    cout << endl << "Total weight of the path:  " << total_weight << endl;

    cout << "\nA* Algorithm::" << endl;
    auto a_star_path = test_graph.a_star({1, 0}, {6, 2});
    cout << "Path found by A*: ";
    total_weight = 0;
    for (size_t i = 0; i < a_star_path.size(); ++i) {
        cout << "(" << a_star_path[i].first << "," << a_star_path[i].second << ") ";
        if (i < a_star_path.size() - 1) {
            for (const auto &neighbor: test_graph.adj_list[a_star_path[i]]) {
                if (neighbor.first == a_star_path[i + 1]) {
                    total_weight += neighbor.second;
                    break;
                }
            }
        }
    }
    cout << endl << "Total weight of the path: " << total_weight << endl;

    cout << "\nBFS::" << endl;
    auto bfs_path = test_graph.bfs({0, 0}, {5, 1});
    cout << "Path found by BFS: ";
    for (const auto &node: bfs_path) {
        cout << "(" << node.first << "," << node.second << ") ";
    }
    cout << endl;

    cout << "\nDFS:" << endl;
    auto dfs_path = test_graph.dfs({0, 0}, {5, 0});
    cout << "Path found by DFS: ";
    for (const auto &node: dfs_path) {
        cout << "(" << node.first << "," << node.second << ") ";
    }
    cout << endl;
    cout << endl;

    Graph g;
    g.load_graph_from_file("C:/Users/andre/CLionProjects/untitled/spb_graph.txt");
    pair<double, double> start{30.296534, 59.959784};
    pair<double, double> end{30.312388, 59.956287};
    measure_dijkstra(g, start, end);
    measure_a_star(g, start, end);
    measure_bfs(g, start, end);
    measure_dfs(g, start, end);

    return 0;
}
