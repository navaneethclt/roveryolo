#include <iostream>
#include <vector>
#include <queue>

class IterativeMedianFilter {
public:
    IterativeMedianFilter(int windowSize) : windowSize(windowSize) {};
    double insertAndCalculateMedian(int newValue);
private:
    void balanceHeaps();
    int windowSize;
    std::queue<int> data;

    std::priority_queue<int, std::vector<int>, std::greater<int>> minHeap;
    std::priority_queue<int, std::vector<int>, std::less<int>> maxHeap;
};