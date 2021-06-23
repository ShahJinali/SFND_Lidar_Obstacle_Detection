/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insertRec(Node **node, std::vector<float> point, int id, unsigned int depth){

	    if(*node == NULL){
	        *node = new Node(point,id);
	        return;
	    }

	    unsigned int A = depth % 2; //K = 2
	    if(point[A] < (*node)->point[A] ){
            insertRec(&(*node)->left,point,id,depth+1);
	    }
	    else{
            insertRec(&(*node)->right,point,id,depth+1);
	    }
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
        insertRec(&root,point,id,0);
	}

    void searchRec(std::vector<float> target, Node* node, unsigned depth, float distanceTol, std::vector<int> &ids) {

        if (node != NULL) {

            /*
            If node is within the box/area of target point, then only check the distance,
            Split the tree, until it reaches to the node that is NULL
            */
            unsigned int A = depth % 2; //K = 2

            //Point within the box
            if ((node->point[0] >= (target[0] - distanceTol)) && (node->point[0] <= (target[0] + distanceTol)) && (node->point[1] >= (target[1] - distanceTol)) && (node->point[1] <= (target[1] + distanceTol))) {

                float distance = sqrt(pow((target[0] - node->point[0]),2) + pow((target[1] - node->point[1]), 2));
                if (distance <= distanceTol) {
                    ids.push_back(node->id);
                }
            }

            if ((target[A] - distanceTol) < node->point[A])
                searchRec(target, node->left, depth + 1, distanceTol, ids);

            if((target[A] + distanceTol) > node->point[A])
                searchRec(target, node->right, depth + 1, distanceTol, ids);

        }
    }

    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol)
    {
        std::vector<int> ids;
        searchRec(target, root, 0, distanceTol, ids);
        return ids;
    }
};




