import java.util.*;

public class Main
{
    public static void main(String[] args)
    {
        Graph graph = new Graph();
        //graph = createLinkedList(10);
        graph.addNode(0);
        graph.addNode(1);
        graph.addNode(2);
        graph.addNode(3);
        graph.addNode(4);
        graph.addNode(5);
        graph.addUndirectedEdge(graph.allNodes.get(0), graph.allNodes.get(1));
        graph.addUndirectedEdge(graph.allNodes.get(0), graph.allNodes.get(2));
        graph.addUndirectedEdge(graph.allNodes.get(1), graph.allNodes.get(3));
        graph.addUndirectedEdge(graph.allNodes.get(3), graph.allNodes.get(5));
        //graph.addUndirectedEdge(graph.allNodes.get(4), graph.allNodes.get(5));
        //graph = createRandomUnweightedGraphIter(10);
        ArrayList<GraphNode> path = GraphSearch.DFSIter(graph, graph.allNodes.get(3), graph.allNodes.get(4));
        System.out.println("************");
        for (GraphNode x : path)
            System.out.println(x.value);
    }

    static Graph createRandomUnweightedGraphIter(int n)
    {
        Graph graph = new Graph();
        for (int i = 0; i < n; i++)
        {
            graph.addNode(i);
        }
        // Between each two nodes, give a 50/50 chance of putting an edge in between
        ArrayList<GraphNode> nodes = graph.getAllNodes();
        for(int i = 0; i < nodes.size(); i++) {
            for (int j = i; j < nodes.size(); j++) {
                Random rand = new Random();
                int add = rand.nextInt(2);
                if (add==1)
                    graph.addUndirectedEdge(nodes.get(i), nodes.get(j));
            }
        }
        return graph;
    }

    static Graph createLinkedList(int n)
    {
        Graph graph = new Graph();
        for (int i = 0; i < n; i++)
        {
            graph.addNode(i);
        }
        // Attach edge between one node and the next only
        ArrayList<GraphNode> nodes = graph.getAllNodes();
        for (int i = 0; i < n-1; i++)
        {
            graph.addUndirectedEdge(nodes.get(i), nodes.get(i+1));
        }

        return graph;
    }

    static ArrayList<GraphNode> BFTRecLinkedList()
    {
        Graph graph = createLinkedList(100);
        return GraphSearch.BFTRec(graph);
    }

    static ArrayList<GraphNode> BFTIterLinkedList()
    {
        Graph graph = createLinkedList(10000);
        return GraphSearch.BFTIter(graph);
    }
}

class Graph
{
    ArrayList<GraphNode> allNodes;
    public Graph() {
         allNodes = new ArrayList<>();
    }

    public void addNode(final int nodeVal)
    {
        GraphNode node = new GraphNode(nodeVal);
        allNodes.add(node);
    }

    public void addUndirectedEdge(final GraphNode first, final GraphNode second)
    {
        Edge edge = new Edge(second);
        first.neighbors.add(edge);
        Edge edge2 = new Edge(first);
        second.neighbors.add(edge2);
    }

    public void removeUndirectedEdge(final GraphNode first, final GraphNode second)
    {
        for (Edge edge : first.neighbors) {
            if (edge.destination == second) {
                first.neighbors.remove(edge);
                break;
            }
        }
        for (Edge edge : second.neighbors) {
            if (edge.destination == first) {
                second.neighbors.remove(edge);
                break;
            }
        }
    }

    public ArrayList<GraphNode> getAllNodes()
    {
        return allNodes;
    }
}

class GraphNode
{
    int value;
    boolean visited;
    ArrayList<Edge> neighbors = new ArrayList<>();
    public GraphNode (int val)
    {
        value = val;
        visited = false;
    }
}

class Edge
{
    int weight;
    GraphNode destination;
    public Edge (GraphNode second)
    {
        weight = 1;
        destination = second;
    }
}

class GraphSearch
{
    static ArrayList<GraphNode> DFSRec(final GraphNode start, final GraphNode end)
    {
        ArrayList<GraphNode> path = new ArrayList<>();
        Stack<GraphNode> stack = new Stack<>();
        stack = DFSRecHelper(start, end, path, stack);

        //Add everything from stack to "path" in reverse
        while(!stack.isEmpty())
            path.add(0, stack.pop());
        return path;

    }
    static Stack<GraphNode> DFSRecHelper(GraphNode start, GraphNode end, ArrayList<GraphNode> path, Stack<GraphNode> stack)
    {
        stack.push(start);
        //Once the end is found, return the path
        if (!stack.isEmpty() && stack.peek() == end) {
            return stack;
        }

        //If node has already been visited, keep going
        if (start.visited) {
            stack.pop();
            return stack;
        }
        //Set visited to true and check neighbors
        start.visited = true;
        for (Edge edge : start.neighbors) {
            DFSRecHelper(edge.destination, end, path, stack);
            if (stack.peek() == end)
                break;
        }
        //if you see the end, search is complete
        if(stack.peek() != end)
            stack.pop();
        return stack;
    }

    static ArrayList<GraphNode> DFSIter(Graph graph, final GraphNode start, final GraphNode end)
    {
        Stack<GraphNode> stack = new Stack<>();
        ArrayList<GraphNode> path = new ArrayList<>();
        GraphNode curr = start;

        // Keep going until you find the end node
        while (curr != end)
        {
            path.add(curr);
            // If already visited, continue
            if (curr.visited)
            {
                //curr = stack.pop();
                path.remove(path.size()-1);
                continue;
            }
            // If disconnect from graph, remove from path
            if (curr.neighbors.isEmpty())
            {
                path.remove(path.size()-1);
            }

            curr.visited = true;
            int count=0;
            // Add each edge to the stack
            for (Edge edge : curr.neighbors)
            {
                if (!edge.destination.visited) {
                    stack.push(edge.destination);
                    count++;
                    continue;
                }
            }
            // If all the neighbors have been visited, remove the curr node from the path (not in the path)
            if (count==0)
                path.remove(path.size()-1);
            if (stack.isEmpty())
                return null;
            curr = stack.pop();
        }
        path.add(curr);
        return path;
    }

    static ArrayList<GraphNode> BFTRec(Graph graph)
    {
        ArrayList<GraphNode> path = new ArrayList<>();
        LinkedList<GraphNode> queue = new LinkedList<>();
        ArrayList<Boolean> visited = new ArrayList<>();

        // Keep track of how many unvisited nodes there are
        for (GraphNode node : graph.getAllNodes())
            visited.add(false);
        graph.getAllNodes().get(0).visited = true;
        visited.remove(0);

        queue.add(graph.getAllNodes().get(0));
        return BFTRecHelper(graph, queue, path, visited);
    }
    static ArrayList<GraphNode> BFTRecHelper(Graph graph, LinkedList<GraphNode> queue, ArrayList<GraphNode> path, ArrayList<Boolean> visited)
    {
        // If all nodes have been visited and nothing left to check, finished
        if (queue.isEmpty() && visited.isEmpty())
            return path;

        // Check nodes that aren't connected to graph
        if (queue.isEmpty() && !visited.isEmpty())
        {
            for (int i = 0; i < graph.getAllNodes().size(); i++)
            {
                if (!graph.getAllNodes().get(i).visited)
                {
                    queue.add(graph.getAllNodes().get(i));
                }
            }
        }
        GraphNode curr = queue.poll();
        path.add(curr);

        // Add all the neighbors to the queue if they've not been visited
        if (!curr.neighbors.isEmpty()) {
            for (Edge edge : curr.neighbors) {
                if (!edge.destination.visited) {
                    edge.destination.visited = true;
                    visited.remove(0);
                    queue.push(edge.destination);
                }
            }
            BFTRecHelper(graph, queue, path, visited);
        }
        return path;
    }

    static ArrayList<GraphNode> BFTIter(final Graph graph)
    {
        ArrayList<GraphNode> path = new ArrayList<>();
        ArrayList<Boolean> visited = new ArrayList<>();
        LinkedList<GraphNode> queue = new LinkedList<>();
        for (int i = 0; i < graph.allNodes.size(); i++)
            visited.add(false);

        graph.allNodes.get(0).visited = true;
        visited.remove(0);
        GraphNode curr = graph.getAllNodes().get(0);
        path.add(curr);

        while (visited.size() != 0) //All nodes not visited
        {
            // Add neighbors to the queue
            if (!curr.neighbors.isEmpty()) {
                for (int i = 0; i < curr.neighbors.size(); i++) {
                    if (curr.neighbors.get(i).destination.visited) {
                        continue;
                    }
                    curr.neighbors.get(i).destination.visited = true;
                    visited.remove(0);
                    queue.add(curr.neighbors.get(i).destination);
                    path.add(curr.neighbors.get(i).destination);
                }
            }
            curr = queue.poll();
            // If no more nodes to visit in graph (connected), find another set of nodes disconnected
            if (queue.isEmpty())
            {
                for (int p = 0; p < graph.getAllNodes().size(); p++)
                {
                    if (!graph.getAllNodes().get(p).visited)
                    {
                        curr = graph.getAllNodes().get(p);
                        path.add(graph.getAllNodes().get(p));
                        graph.getAllNodes().get(p).visited = true;
                        visited.remove(0);
                        break;
                    }
                }
            }
        }
        return path;
    }
}