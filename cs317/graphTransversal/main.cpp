/*
 * Graph traversal program
 * CS317 - Fall 2014
 * By: Christopher Bero
 *
 */

#include <iostream>
#include <stdio.h>
#include <stack>
#include <queue>

using namespace std;

/*
 * Print graph
 */
void enumGraph (int _arr[][9], int _len1, int _len2) {
	cout << endl << "Graph - " << _len1 << " : " << _len2 << endl;
	for (int c = 0; c < _len1; c++) { // First dimension - nodes
			cout << endl << c << ": ";
		for (int d = 0; d < _len2; d++) { // Second dimension - adjacency list
			if (_arr[c][d] == -1) {
				break; // Don't print init values
			}
			//cout << _arr[c][d] << " ";
			printf("%4d", _arr[c][d]); // Better formatting
		}
	}
}

/*
 * Determine if node has been previously researched
 */
bool isDiscovered (stack<int> disc, int vert) {
	while (!disc.empty()) {
		if (vert == disc.top()) {
			return true;
		}
		disc.pop();
	}
	return false;
}

/*
 * Traverse Depth-first, print results to cout
 */
void runDepth (int arr[][9], int vert) {
	stack<int> mainStack;
	stack<int> discoveryStack;

	mainStack.push(vert);

	while (!mainStack.empty()) {
		int temp = mainStack.top();
		mainStack.pop();

		if (!isDiscovered(discoveryStack, temp)) {
			discoveryStack.push(temp);
			for (int c = 0; c < 9; c++) {
				if (arr[temp][c] == -1) {
					break;
				}
				mainStack.push(arr[temp][c]);
			}
		}
	}

	cout << endl << endl << "runDepth done: " << endl;
	while (!discoveryStack.empty()) {
		cout << discoveryStack.top() << ", ";
		discoveryStack.pop();
	}

}

/*
 * Traverse Breadth-first, print results to cout
 */
void runBreadth (int arr[][9], int vert) {
	queue<int> nodes;
	stack<int> discoveryStack;

	nodes.push(vert);
	discoveryStack.push(vert);

	while (!nodes.empty()) {
		int temp = nodes.front();
		nodes.pop();

		for (int c = 0; c < 9; c++) {
			if (arr[temp][c] == -1) {
				break;
			}
			if (!isDiscovered(discoveryStack, arr[temp][c])) {
				discoveryStack.push(arr[temp][c]);
				nodes.push(arr[temp][c]);
			}
		}
	}

	stack<int> result;

	while (!discoveryStack.empty()) {
		result.push(discoveryStack.top());
		discoveryStack.pop();
	}

	cout << endl << endl << "runBreadth done: " << endl;
	while (!result.empty()) {
		cout << result.top() << ", ";
		result.pop();
	}

}

int main()
{
	// Main data structure
    int graph [10][9] = {-1};

	// Initialize graph
	for (int c = 0; c < 10; c++) {
		for (int d = 0; d < 9; d++) {
			graph[c][d] = -1;
		}
	}

	// Print graph with -1's
	cout << endl << "Initial Graph - 10 : 9 " << endl;
	for (int c = 0; c < 10; c++) {
			cout << endl << c << ": ";
		for (int d = 0; d < 9; d++) {
			printf("%4d", graph[c][d]); // Better formatting
		}
	}

	// Print instructions
	cout << endl << endl << "Node 0 is the anchor for traversal."
	<< endl << "For each of the following nodes, enter an adjacent node number (0-9)." <<
	"Enter -1 to move to the next node." << endl
	<< "Enter -2 to be done entirely." << endl << endl;

	// Accept input and run basic pruning
	for (int c = 0; c < 10; c++) {
		for (int d = 0; d < 9; d++) {
			int temp = 0;
			cout << "Enter adjacent node to " << c << ": ";
			cin >> temp;
			if (temp == -2) {
				c = 11;
				d = 11;
				break;
			} else if (temp == -1) {
				break;
			} else if ((temp == c) || (temp > 9) || (temp < 0)) {
				d--;
				continue;
			}
			for (int t = 0; t < d; t++) {
				if (graph[c][t] == temp) {
					d--;
					continue;
				}
			}
			graph[c][d] = temp;
		}
	}

	// Print user-defined adjacency list
	enumGraph(graph, 10, 9);

	// Traverse Depth-first
	runDepth(graph, 0);

	// Traverse Breadth-first
	runBreadth(graph, 0);

    return 0;
}
