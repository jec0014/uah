/*
 * QuickSort program
 * CS317 Fall-2014
 * By: Christopher Bero
 *
 * This program took about ten minutes to debug, which was much faster thanks
 * to the debugging tools AJ discussed with us in class. I used step-into and step-outof
 * to locate a bad conditional value in partitioner().
 *
 */

#include <iostream>

using namespace std;

void enumArr (int * _arr, int _len) {
	cout << endl << "array - " << _len << " ";
	for (int c = 0; c < _len; c++) {
		cout << ":" << _arr[c];
	}
}

int partitioner (int * arr, int first, int last) {
	int left = first + 1;
	int right = last;
	int pivot = arr[first];
	bool breakFlag = false;

	while (breakFlag == false) {
		while ((arr[left] >= pivot) && (left <= right)) {
			left++;
		}
		while ((arr[right] <= pivot) && (left <= right)) {
			right--;
		}

		if (right < left) {
			breakFlag = true;
		} else {
			int temp = 0;
			temp = arr[right];
			arr[right] = arr[left];
			arr[left] = temp;
		}

	}
	arr[first] = arr[right];
	arr[right] = pivot;

	return right;

}

void quickSorter (int * arr, int first, int last) {
	cout << endl << "quickSorter: " << first << " : " << last;
	enumArr(arr, (last-first));
	int pivot = 0;
	if (first < last) {
	pivot = partitioner(arr, first, last);
	quickSorter(arr, first, (pivot-1));
	quickSorter(arr, (pivot+1), last);
	}
}

int main()
{
    int toSort [] = {1,3,2,6,4,5,9,7,8};
	int toSortLen = (sizeof(toSort)/sizeof(*toSort));

	quickSorter(toSort, 0, (toSortLen-1));

	coud << endl << "Done: " << endl;
	enumArr(toSort, toSortLen);

    return 0;
}
