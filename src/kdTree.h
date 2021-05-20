// KdTree structure and methods
#include <vector>
#include <math.h>

// node
struct Node
{
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;

    // constructor with initialization
    Node(std::vector<float> arr, int setID) : point(arr), id(setID), left(NULL), right(NULL) {}

    // destructor
    ~Node()
    {
        delete left;
        delete right;
    }
};

// tree
struct KdTree
{
    // constructor
    KdTree() : root(NULL){}
    // destructor
    ~KdTree()
    {
        delete root;
    }
    // root node pointer
    Node* root;

    // methods
    void insert(std::vector<float> point, int id)
    {
        insertRecursively(&root, 0, point, id);
    }

    void insertRecursively(Node** node, unsigned int depth, std::vector<float> point, int id)
    {
        // if no root node, create a new node
        if(*node == NULL)
            *node = new Node(point, id);
        else{
            // calculate depth TODO: add initialization support for k dimensions
            unsigned int currentDepth = depth % 3;
            // if less than go left
            if(point[currentDepth] < ((*node)->point[currentDepth]))
                insertRecursively(&(*node)->left, depth+1, point, id);
            // if greater than go right
            else    
                insertRecursively(&(*node)->right, depth+1, point, id);
        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTolerance)
    {
        std::vector<int> ids;
        searchRecursively(target, &root, 0, distanceTolerance, ids);
        return ids;
    }

    void searchRecursively(std::vector<float> target, Node** node, unsigned int depth, float distanceTolerance, std::vector<int> &ids)
    {
        if(*node != NULL)
        {
            // check x, y, and z and calculate distance from 
            if((*node)->point[0] >= (target[0] - distanceTolerance) && (*node)->point[0] <= (target[0] + distanceTolerance)
                && (*node)->point[1] >= (target[1] - distanceTolerance) && (*node)->point[1] <= (target[1] + distanceTolerance)
                && (*node)->point[2] >= (target[2] - distanceTolerance) && (*node)->point[2] <= (target[2] + distanceTolerance))
            {
                double distance = sqrt(((*node)->point[0] - target[0])*((*node)->point[0] - target[0])
                                      + ((*node)->point[1] - target[1])*((*node)->point[1] - target[1])
                                      + ((*node)->point[2] - target[2])*((*node)->point[2] - target[2]));
                if(distance <= distanceTolerance)
                    ids.push_back((*node)->id);
            }
            // check left or right
            if((target[depth % 3] - distanceTolerance) <= (*node)->point[depth % 3])
                searchRecursively(target, &((*node)->left), depth + 1, distanceTolerance, ids);
            if((target[depth % 3] + distanceTolerance) >= (*node)->point[depth % 3])
                searchRecursively(target, &((*node)->right), depth + 1, distanceTolerance, ids);
        }
        
    }
};
