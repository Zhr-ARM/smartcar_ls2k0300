#ifndef _class_thread_wrapper_class_h_
#define _class_thread_wrapper_class_h_


#include <atomic>
#include <pthread.h>
#include <iostream>
#include <functional>
#include <sys/timerfd.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <inttypes.h>
#include <thread>
#include <chrono>

#include "zf_common_headfile.h"

class timer_fd 
{
public:
    struct ThreadInfo
    {
        int32 tid;
        int32 policy;
        int32 priority;
        bool valid;
    };

    timer_fd(int interval, const std::function<void()>& func)
        : interval(interval), func(func), running(false),
          thread_tid_(0), thread_policy_(0), thread_priority_(0) {}

    ~timer_fd() {
        stop();
    }

    void start() 
    {
        if (running) return;
        running = true;
        timerThread = std::thread(&timer_fd::timer_loop, this);
    }

    void stop() {
        if (!running) return;
        running = false;
        if (timerThread.joinable()) {
            timerThread.join();
        }
    }

    ThreadInfo thread_info() const
    {
        ThreadInfo info;
        info.tid = thread_tid_.load();
        info.policy = thread_policy_.load();
        info.priority = thread_priority_.load();
        info.valid = (0 < info.tid);
        return info;
    }

private:

    int interval;
    std::function<void()> func;
    std::thread timerThread;
    bool running;
    std::atomic<int32> thread_tid_;
    std::atomic<int32> thread_policy_;
    std::atomic<int32> thread_priority_;

    int init_timer_fd() 
    {
        int fd = timerfd_create(CLOCK_MONOTONIC, 0);
        if (fd == -1) {
            perror("timerfd_create");
            return -1;
        }

        struct itimerspec its;
        its.it_value.tv_sec = 0;
        its.it_value.tv_nsec = interval * 1000000;
        its.it_interval.tv_sec = 0;
        its.it_interval.tv_nsec = interval * 1000000;
 
        if (timerfd_settime(fd, 0, &its, NULL) == -1) {
            perror("timerfd_settime");
            close(fd);
            return -1;
        }

        return fd;
    }

    void timer_loop()
    {
        int fd = init_timer_fd();
        if (fd == -1) return;

        struct sched_param param;
        param.sched_priority = 10;
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

        {
            int policy = 0;
            struct sched_param current_param;
            memset(&current_param, 0, sizeof(current_param));

            thread_tid_ = (int32)syscall(SYS_gettid);
            if (0 == pthread_getschedparam(pthread_self(), &policy, &current_param))
            {
                thread_policy_ = policy;
                thread_priority_ = current_param.sched_priority;
            }
            else
            {
                thread_policy_ = 0;
                thread_priority_ = 0;
            }
        }

        uint64_t expirations;
        while (running) 
        {
            if (read(fd, &expirations, sizeof(expirations)) != sizeof(expirations)) 
            {
                perror("read");
                break;
            }

            // 只执行一次回调，丢弃累积的过期次数，避免雪崩式补偿
            if (expirations > 0)
            {
                func();
            }
        }
        close(fd);
    }
};

    
#endif
