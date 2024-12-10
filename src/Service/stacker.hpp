#pragma once

#include <iostream>
#include <mutex>
#include <vector>

template <typename T>
class circular_stacker {
public:
    circular_stacker(size_t size) : size_(size), buffer_(size) {
        head_ = tail_ = count_ = newest_ = 0;
        is_overwrite_ = false;
    }

    ~circular_stacker() {
        buffer_.clear();
    }

    // スタックの名前を設定
    void set_name(std::string name) {
        name_ = name;
    }

    // 要素を追加
    void push(const T& value) {
        std::lock_guard<std::mutex> lock(mtx_);

        if (is_overwrite_) {
            buffer_[tail_] = value;
            newest_ = tail_;
            tail_ = (tail_ + 1) % size_;
            if (count_ < size_) {
                count_++;
            } else {
                head_ = (head_ + 1) % size_;
            }
        } else {
            if (count_ < size_) {
                buffer_[tail_] = value;
                tail_ = (tail_ + 1) % size_;
                count_++;
            } else {
                std::cerr
                    << name_.c_str()
                    << "Circular stacker is full. Cannot push more elements."
                    << std::endl;
            }
        }
    }

    // 要素を取得
    T pop() {
        std::lock_guard<std::mutex> lock(mtx_);
        if (count_ > 0) {
            T value = buffer_[head_];
            head_ = (head_ + 1) % size_;
            count_--;
            return value;
        } else {
            std::cerr << name_.c_str()
                      << "Circular stacker is empty. Cannot pop elements."
                      << std::endl;
            return T();  // デフォルト値を返す
        }
    }

    // 先頭の要素を取得
    T front() {
        std::lock_guard<std::mutex> lock(mtx_);
        if (count_ > 0) {
            return buffer_[head_];
        } else {
            std::cerr << name_.c_str()
                      << "Circular stacker is empty. Cannot get front element."
                      << std::endl;
            return T();  // デフォルト値を返す
        }
    }

    // 末尾の要素を取得
    T back() {
        std::lock_guard<std::mutex> lock(mtx_);
        if (count_ > 0) {
            return buffer_[newest_];
        } else {
            std::cerr << name_.c_str()
                      << "Circular stacker is empty. Cannot get back element."
                      << std::endl;
            return T();  // デフォルト値を返す
        }
    }

    // 現在の要素数を取得
    size_t size() const {
        return count_;
    }

    // スタックの最大要素数を取得
    size_t max_size() const {
        return size_;
    }

    // スタックをクリア
    void clear() {
        std::lock_guard<std::mutex> lock(mtx_);
        head_ = tail_ = count_ = newest_ = 0;
    }

    void change_overwrite_mode(bool is_overwrite) {
        is_overwrite_ = is_overwrite;
    }

private:
    size_t size_;
    std::vector<T> buffer_;
    size_t head_;
    size_t tail_;
    size_t newest_;  // 最新の要素の位置
    size_t count_;
    std::mutex mtx_;
    bool is_overwrite_;
    std::string name_;
};