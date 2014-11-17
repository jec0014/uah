/*
 * MergeSort program
 * CS317 Fall 2014
 * By: Christopher Bero
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

void merger (int * subA, int subALen, int * subB, int subBLen, int * toSort, int toSortLen) {
	int indexA = 0;
	int indexB = 0;
	int indexSort = 0;
	while ((indexA < subALen) && (indexB < subBLen)) {
		if (subA[indexA] <= subB[indexB]) {
			toSort[indexSort] = subA[indexA];
			indexA++;
		} else {
			toSort[indexSort] = subB[indexB];
			indexB++;
		}
		indexSort++;
	}

	if (indexA == subALen) {
		for (int c = indexSort; c < toSortLen; c++) {
			toSort[indexSort] = subB[indexB];
			indexSort++;
			indexB++;
		}
	} else {
		for (int c = indexSort; c < toSortLen; c++) {
			toSort[indexSort] = subA[indexA];
			indexSort++;
			indexA++;
		}
	}

}

void divider (int * _toSort, int _toSortLen) {
	if (!(_toSortLen > 1)) {
		return;
	}

	int subA [(_toSortLen/2)];
	int subB [((_toSortLen+1)/2)];
	int subALen = (sizeof(subA)/sizeof(*subA));
	int subBLen = (sizeof(subB)/sizeof(*subB));

	for (int c = 0; c < _toSortLen; c++) {
		if (c >= subALen) {
			subB[c-subALen] = _toSort[c];
		} else {
			subA[c] = _toSort[c];
		}
	}

	divider(subA, subALen);
	divider(subB, subBLen);

	cout << endl << "instance: ";
	enumArr(subA, subALen);
	enumArr(subB, subBLen);
	cout << endl;

	merger(subA, subALen, subB, subBLen, _toSort, _toSortLen);

}

int main()
{

	int toSort [] = {1,3,2,6,4,5,9,7,8};
	int toSortLen = (sizeof(toSort)/sizeof(*toSort));

    divider(toSort, toSortLen);

	cout << endl << "done: " << endl;
	enumArr(toSort, toSortLen);

    return 0;
}
