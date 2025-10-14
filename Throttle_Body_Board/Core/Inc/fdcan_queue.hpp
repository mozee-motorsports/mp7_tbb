// error_buffer.hpp
#ifndef FDCAN_BUFFER_HPP
#define FDCAN_BUFFER_HPP

extern "C" {
	#include "fdcan.h"
}


class FDCANBuffer {
public:
    explicit FDCANBuffer(size_t capacity)
        : capacity_(capacity), buffer_(new CANMessage[capacity]), head_(0), tail_(0), count_(0) {}

    ~FDCANBuffer() {
        delete[] buffer_;
    }

    // Add an error, overwrite oldest if full, but only if it’s not already present
    bool push(const CANMessage& msg) {
        // Check if the error already exists in the buffer
//        for (size_t i = 0; i < count_; i++) {
//            size_t pos = (tail_ + i) % capacity_;
//            if (buffer_[pos].code == err.code && buffer_[pos].timestamp == err.timestamp) {
//                return false; // Duplicate found, don’t add
//            }
//        }

        // No duplicate found, add the error
        buffer_[head_] = msg;
        head_ = (head_ + 1) % capacity_;
        if (count_ < capacity_) {
        	size_t temp = count_;
        	temp = temp + 1;
            count_ = temp;
        } else {
            tail_ = (tail_ + 1) % capacity_; // Overwrite oldest
        }
        return true; // Successfully added
    }

    // Get error at index (0 = oldest, count-1 = newest), returns false if index invalid
    bool get(size_t index, CANMessage& out) const {
        if (index >= count_) return false;
        size_t pos = (tail_ + index) % capacity_;
        out = buffer_[pos];
        return true;
    }

    // Number of errors currently stored
    size_t size() const { return count_; }

    // Maximum capacity
    size_t capacity() const { return capacity_; }

    // Clear the buffer
    void clear() {
        head_ = 0;
        tail_ = 0;
        count_ = 0;
    }

private:
    size_t capacity_;
    CANMessage* buffer_;
    volatile size_t head_;  // Next position to write
    volatile size_t tail_;  // Oldest error position
    volatile size_t count_; // Current number of errors
};

#endif // FDCAN_BUFFER_HPP
