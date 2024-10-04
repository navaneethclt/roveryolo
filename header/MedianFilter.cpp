#include <iostream>
#include <vector>
#include <queue>

class IterativeMedianFilter {
public:
    IterativeMedianFilter(int windowSize) : windowSize(windowSize) {}

    void insert(int newValue) {
        window.push(newValue);

        if (window.size() > windowSize) {
            int removedValue = window.front();
            window.pop();
            if (removedValue <= minHeap.top()) {
                maxHeap.push(removedValue);
            }
            else {
                minHeap.push(removedValue);
            }
            balanceHeaps();
        }

        if (window.size() == windowSize) {
            if (minHeap.empty() || newValue <= minHeap.top()) {
                maxHeap.push(newValue);
            }
            else {
                minHeap.push(newValue);
            }
            balanceHeaps();
        }
    }

    double getMedian() {
        if (windowSize % 2 == 0) {
            return (static_cast<double>(maxHeap.top()) + static_cast<double>(minHeap.top())) / 2.0;
        }
        else {
            return static_cast<double>(minHeap.top());
        }
    }

private:
    void balanceHeaps() {
        while (maxHeap.size() > minHeap.size() + 1) {
            minHeap.push(maxHeap.top());
            maxHeap.pop();
        }

        while (minHeap.size() > maxHeap.size()) {
            maxHeap.push(minHeap.top());
            minHeap.pop();
        }
    }

    int windowSize;
    std::priority_queue<int> maxHeap;
    std::priority_queue<int, std::vector<int>, std::greater<int>> minHeap;
    std::queue<int> window;
};