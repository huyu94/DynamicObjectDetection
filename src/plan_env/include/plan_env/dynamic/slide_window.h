#include <deque>

template <typename T>
class SlidingWindow
{
public:
    SlidingWindow(int window_size) : size_(window_size) {}
    ~SlidingWindow() {}

    void add(T value)
    {
        if(windows_.size() >= size_)
        {
            windows_.pop_front();
        }
        window_.push_back(value);
    }


    std::queue<T> get() const 
    {
        return window_;
    }


private:
    std::queue<int> window_;
    int size_;
};