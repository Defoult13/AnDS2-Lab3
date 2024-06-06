#include <vector>
#include <unordered_map>
#include <algorithm>
#include <iterator>
#include <iostream>
#include <queue>

template<typename Vertex, typename Distance = double>
class Graph {
public:
    struct Edge {
        Vertex from;
        Vertex to;
        Distance distance;
        Edge(const Vertex& f, const Vertex& t, const Distance& d) : from(f), to(t), distance(d) {}
    };
private:
    std::unordered_map<Vertex, std::vector<Edge>> _graph;
public:
    bool has_vertex(const Vertex& v) const {
        return _graph.find(v) != _graph.end();
    }
    void add_vertex(const Vertex& v) {
        if (!has_vertex(v)) {
            _graph[v] = std::vector<Edge>();
        }
    }
    bool delete_vertex(const Vertex& v) {
        auto it = _graph.find(v);
        if (it != _graph.end()) {
            _graph.erase(it);
            for (auto& pair : _graph) {
                auto& edges = pair.second;
                edges.erase(std::remove_if(edges.begin(), edges.end(), [&v](const Edge& e) {
                    return e.to == v;
                    }), edges.end());
            }
            return true;
        }
        return false;
    }
    std::vector<Vertex> vertices() const {
        std::vector<Vertex> result;
        for (const auto& pair : _graph) {
            result.push_back(pair.first);
        }
        return result;
    }
    void add_edge(const Vertex& from, const Vertex& to, const Distance& d) {
        if (has_vertex(from) && has_vertex(to)) {
            _graph[from].push_back(Edge(from, to, d));
        }
    }
    bool delete_edge(const Vertex& from, const Vertex& to) {
        auto it = _graph.find(from);
        if (it != _graph.end()) {
            auto& edges = it->second;
            auto edge_it = std::remove_if(edges.begin(), edges.end(), [&to](const Edge& e) {return e.to == to; });
            if (edge_it != edges.end()) {
                edges.erase(edge_it, edges.end());
                return true;
            }
        }
        return false;
    }
    bool delete_edge(const Edge& e) {
        auto it = _graph.find(e.from);
        if (it != _graph.end()) {
            auto& edges = it->second;
            auto edge_it = std::remove_if(edges.begin(), edges.end(), [&e](const Edge& edge) {
                return edge.from == e.from && edge.to == e.to && edge.distance == e.distance;
                });
            if (edge_it != edges.end()) {
                edges.erase(edge_it, edges.end());
                return true;
            }
        }
        return false;
    }
    bool has_edge(const Vertex& from, const Vertex& to) const {
        auto it = _graph.find(from);
        if (it != _graph.end()) {
            const auto& edges = it->second;
            return std::any_of(edges.begin(), edges.end(), [&to](const Edge& e) {return e.to == to; });
        }
        return false;
    }
    bool has_edge(const Edge& e) const {
        auto it = _graph.find(e.from);
        if (it != _graph.end()) {
            const auto& edges = it->second;
            return std::any_of(edges.begin(), edges.end(), [&e](const Edge& edge) {
                return edge.from == e.from && edge.to == e.to && edge.distance == e.distance;
                });
        }
        return false;
    }
    std::vector<Edge> edges(const Vertex& vertex) {
        auto it = _graph.find(vertex);
        if (it != _graph.end()) {
            return it->second;
        }
        return std::vector<Edge>();
    }
    size_t order() const {
        return _graph.size();
    }
    size_t degree(const Vertex& v) const {
        auto it = _graph.find(v);
        if (it != _graph.end()) {
            return it->second.size();
        }
        return 0;
    }
    std::vector<Edge> shortest_path(const Vertex& from, const Vertex& to) const {
        std::unordered_map<Vertex, Distance> dist;
        std::unordered_map<Vertex, Vertex> predecessor;
        for (const auto& vertex : vertices()) {
            dist[vertex] = std::numeric_limits<Distance>::infinity();
            predecessor[vertex] = Vertex();
        }
        dist[from] = 0;
        for (size_t i = 0; i < order() - 1; ++i) {
            for (const auto& vertex : vertices()) {
                auto it = _graph.find(vertex);
                if (it != _graph.end()) {
                    const auto& edges = it->second;
                    for (const auto& edge : edges) {
                        if (dist[vertex] + edge.distance < dist[edge.to]) {
                            dist[edge.to] = dist[vertex] + edge.distance;
                            predecessor[edge.to] = vertex;
                        }
                    }
                }
            }
        }
        std::vector<Edge> path;
        for (Vertex current = to; current != Vertex(); current = predecessor[current]) {
            Vertex pred = predecessor[current];
            path.push_back({ pred, current, dist[current] });
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
    std::vector<Vertex> walk(const Vertex& start_vertex) const {
        std::vector<Vertex> result;
        if (!has_vertex(start_vertex)) {
            return result;
        }
        std::unordered_map<Vertex, bool> visited;
        std::queue<Vertex> queue;

        visited[start_vertex] = true;
        queue.push(start_vertex);
        while (!queue.empty()) {
            Vertex current = queue.front();
            queue.pop();
            result.push_back(current);
            for (const auto& pair : _graph) {
                const auto& edges = pair.second;
                if (pair.first == current) {
                    for (const auto& edge : edges) {
                        if (!visited[edge.to]) {
                            visited[edge.to] = true;
                            queue.push(edge.to);
                        }
                    }
                    break;
                }
            }
        }
        return result;
    }

    Vertex furthest_from_neighbors() const {
        Vertex furthest_vertex;
        Distance max_average_distance = -1;

        for (const auto& pair : _graph) {
            const Vertex& vertex = pair.first;
            const auto& edges = pair.second;

            if (!edges.empty()) {
                Distance total_distance = 0;
                for (const auto& edge : edges) {
                    total_distance += edge.distance;
                }

                Distance average_distance = total_distance / edges.size();
                if (average_distance > max_average_distance) {
                    max_average_distance = average_distance;
                    furthest_vertex = vertex;
                }
            }
        }

        return furthest_vertex;
    }
};

int main() {

    Graph<std::string> graph;

    // Add vertices
    graph.add_vertex("A");
    graph.add_vertex("B");
    graph.add_vertex("C");
    graph.add_vertex("D");

    // Add edges
    graph.add_edge("A", "B", 1.0);
    graph.add_edge("A", "C", 2.0);
    graph.add_edge("B", "C", 1.5);
    graph.add_edge("C", "D", 2.5);

    // Check vertices
    std::cout << "Vertices in the graph: ";
    for (const auto& v : graph.vertices()) {
        std::cout << v << " ";
    }
    std::cout << std::endl;

    // Check edges
    std::cout << "Edges from vertex A: ";
    for (const auto& edge : graph.edges("A")) {
        std::cout << "(" << edge.from << " -> " << edge.to << ", " << edge.distance << ") ";
    }
    std::cout << std::endl;

    // Check if graph has specific vertices and edges
    std::cout << "Graph has vertex A: " << (graph.has_vertex("A") ? "Yes" : "No") << std::endl;
    std::cout << "Graph has edge from A to B: " << (graph.has_edge("A", "B") ? "Yes" : "No") << std::endl;

    // Remove edge and check again
    graph.delete_edge("A", "B");
    std::cout << "Graph has edge from A to B after removal: " << (graph.has_edge("A", "B") ? "Yes" : "No") << std::endl;

    // Shortest path
    auto path = graph.shortest_path("A", "D");
    std::cout << "Shortest path from A to D: ";
    for (const auto& edge : path) {
        std::cout << "(" << edge.from << " -> " << edge.to << ", " << edge.distance << ") ";
    }
    std::cout << std::endl;

    // Walk the graph
    auto walk_result = graph.walk("A");
    std::cout << "Walk starting from A: ";
    for (const auto& v : walk_result) {
        std::cout << v << " ";
    }
    std::cout << std::endl;

    Graph<int, double> g;
    g.add_vertex(1);
    g.add_vertex(2);
    g.add_vertex(3);
    g.add_vertex(4);
    g.add_edge(1, 2, 5.0);
    g.add_edge(2, 1, 5.0);
    g.add_edge(2, 3, 7.0);
    g.add_edge(3, 2, 7.0);
    g.add_edge(3, 4, 10.0);
    g.add_edge(4, 3, 10.0);

    int furthest_vertex = g.furthest_from_neighbors();
    std::cout << "The trauma center furthest from its direct neighbors is: " << furthest_vertex << std::endl;

    return 0;
}