// KdTree structure and methods
#include <vector>

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
struct kdTree
{
    // constructor
    kdTree() : root(NULL){}
    // destructor
    ~kdTree()
    {
        delete root;
    }
    // root node pointer
    Node* root;

    // methods
    void insert(std::vector<float> point, int id)
    {
        
    }

    void insertRecursively(Node** node, unsigned int depth, std::vector<float> point, int id)
    {
        // if no root node, create a new node
        if(*node == NULL)
            *node = new Node(point, id);
        else{
            // calculate depth
            unsigned int currentDepth = depth % 2;
            // if less than go left

            // if greater than go right
        }
    }
};
